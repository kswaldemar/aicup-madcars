//
// Created by valdemar on 05.09.18.
//

#include "cp_helpers.h"
#include "../common/logger.h"

#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_structs.h>

#include <cassert>
#include <cstring>
#include <map>

using namespace cp;

//MARK: Custom allocator

class alloc_control_t {
public:
    friend class cp::Space;

    alloc_control_t();

    void *calloc(size_t nmemb, size_t size);

    void *realloc(void *ptr, size_t size);

    void free(void *ptr);

    inline size_t dump(void *ptr_to) {
        std::memcpy(ptr_to, buffer_.get(), used_bytes_);
        return used_bytes_;
    }

    inline void load(void *ptr_from, size_t bytes) {
        std::memcpy(buffer_.get(), ptr_from, bytes);
        used_bytes_ = bytes;
    }

    void print_usage_statistics() const;

    size_t block_size() const {
        return used_bytes_;
    }

private:
    struct meta_t {
        uint32_t size: 31;
        uint8_t used: 1;
    } __attribute__((packed));

    inline meta_t &get_meta(void *ptr) const {
        return static_cast<meta_t*>(ptr)[-1];
    }

    inline void *find_unused_block(size_t min_size) {
        uint8_t *end = &buffer_[used_bytes_];
        uint8_t *begin = &buffer_[sizeof(meta_t)];
        for (uint8_t *it = begin; it < end;) {
            auto &it_m = get_meta(it);
            if (it_m.used == 0 && it_m.size >= min_size) {
                return it;
            }
            it += it_m.size + sizeof(meta_t);
        }
        return nullptr;
    }

    inline void *allocate_new_block(size_t size) {
        used_bytes_ += sizeof(meta_t);
        void *ret = &buffer_[used_bytes_];
        auto &meta = get_meta(ret);
        meta.size = static_cast<uint32_t>(size);
        meta.used = 1;
        used_bytes_ += meta.size;

        //Zero block, because it may be calloc
        memset(ret, 0, size);

        assert(used_bytes_ < ALLOC_BUF_SIZE);

        return ret;
    }

    size_t used_bytes_ = 0;
    std::unique_ptr<uint8_t[]>(buffer_);
};

alloc_control_t::alloc_control_t() {
    //One huge block
    buffer_.reset(new uint8_t[ALLOC_BUF_SIZE]);
    memset(buffer_.get(), 0, ALLOC_BUF_SIZE);
}

void *alloc_control_t::calloc(size_t nmemb, size_t size) {
    //Try to find unused block with enough size in already allocated memory
    if (void *p = find_unused_block(nmemb * size); p != nullptr) {
        auto &p_m = get_meta(p);
        p_m.used = 1;
        memset(p, 0, p_m.size);
        LOG_V9("Calloc %lu; reuse %p, size = %u", nmemb * size, p, p_m.size);
        return p;
    }

    auto ret = allocate_new_block(nmemb * size);
    LOG_V9("Calloc %lu; new block %p", nmemb * size, ret);
    return ret;
}

void *alloc_control_t::realloc(void *ptr, size_t size) {
    //Check if block is already big enough
    auto &meta = get_meta(ptr);
    if (meta.size >= size) {
        LOG_V9("Realloc %p, %lu; use same pointer, size = %u", ptr, size, meta.size);
        return ptr;
    }

    if (static_cast<uint8_t*>(ptr) + meta.size == &buffer_[used_bytes_]) {
        LOG_V9("Realloc %p, %lu; extend buffer %u", ptr, size, meta.size);
        used_bytes_ += size - meta.size;
        meta.size = static_cast<uint32_t>(size);
        return ptr;
    }

    //Mark memory as free
    meta.used = 0;

    void *ret = nullptr;
    if (void *p = find_unused_block(size); p != nullptr) {
        auto &p_m = get_meta(p);
        p_m.used = 1;
        ret = p;
        LOG_V9("Realloc %p, %lu; reuse %p, size = %u", ptr, size, p, p_m.size);
    } else {
        ret = allocate_new_block(size);
        LOG_V9("Realloc %p, %lu; new block %p", ptr, size, ret);
    }

    //Copy data
    std::memcpy(ret, ptr, meta.size);
    return ret;
}

void alloc_control_t::free(void *ptr) {
    auto &meta = get_meta(ptr);
    meta.used = 0;
    LOG_V9("Free %p; %u bytes", ptr, meta.size);
}

void alloc_control_t::print_usage_statistics() const {
    uint32_t active_memory_bytes = 0;

    uint8_t *end = &buffer_[used_bytes_];
    uint8_t *begin = &buffer_[sizeof(meta_t)];
    for (uint8_t *it = begin; it < end;) {
        const auto &it_m = get_meta(it);
        LOG_V8("meta: %d, size %u", it_m.used, it_m.size);
        it += it_m.size + sizeof(meta_t);
        if (it_m.used) {
            active_memory_bytes += it_m.size;
        }
    }

    LOG_DEBUG("alloc_control_t:: Block size %lu; Active memory %u", used_bytes_, active_memory_bytes);
}

alloc_control_t g_mm;

namespace cp {

transaction_t::transaction_t() {
    ptr.reset(new uint8_t[ALLOC_BUF_SIZE]);
    loose_state[0] = false;
    loose_state[1] = false;
}

size_t memdump(void *ptr_to) {
    return g_mm.dump(ptr_to);
}

void memload(void *ptr_from, size_t bytes) {
    g_mm.load(ptr_from, bytes);
}

void print_memory_usage() {
    return g_mm.print_usage_statistics();
}
} // namespace cp

void *memento_calloc(size_t nmemb, size_t size) {
    return g_mm.calloc(nmemb, size);
}

void *memento_realloc(void *ptr, size_t size) {
    return g_mm.realloc(ptr, size);
}

void memento_free(void *ptr) {
    g_mm.free(ptr);
}

//MARK: Space

Space::Space() {
    impl_ = cpSpaceNew();
}

Space::~Space() {
    g_mm.print_usage_statistics();
    clear();
    cpSpaceFree(impl_);
}

cpSpace *Space::native() const {
    return impl_;
}

void Space::add_shape(cpShape *shape) {
    assert(!readonly_);
    shapes_.push_back(shape);
    cpSpaceAddShape(impl_, shape);
}

void Space::add_body(cpBody *body) {
    assert(!readonly_);
    bodies_.push_back(body);
    cpSpaceAddBody(impl_, body);
}

void Space::add_constraint(cpConstraint *constraint) {
    assert(!readonly_);
    constraints_.push_back(constraint);
    cpSpaceAddConstraint(impl_, constraint);
}

void Space::clear() {
    readonly_ = false;

    for (auto o : shapes_) {
        cpSpaceRemoveShape(impl_, o);
        cpShapeFree(o);
    }
    shapes_.clear();
    for (auto o : bodies_) {
        cpSpaceRemoveBody(impl_, o);
        cpBodyFree(o);
    }
    bodies_.clear();
    for (auto o : constraints_) {
        cpSpaceRemoveConstraint(impl_, o);
        cpConstraintFree(o);
    }
    constraints_.clear();
}

cpBody *Space::static_body() const {
    return cpSpaceGetStaticBody(impl_);
}
