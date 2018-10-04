//
// Created by valdemar on 05.09.18.
//

#pragma once

#include <chipmunk/chipmunk.h>

#include <vector>
#include <memory>
#include <map>

namespace cp {

///Custom allocator memory management
inline constexpr size_t ALLOC_BUF_SIZE = 256 * 1024;

struct transaction_t {
    transaction_t();

    std::unique_ptr<uint8_t[]> ptr;
    size_t bytes;
    uint16_t ticks_to_deadline;
    bool loose_state[2];
    bool in_air_state[2];
    int turn_idx;
};

size_t memdump(void *ptr_to);
void memload(void *ptr_from, size_t bytes);

void print_memory_usage();

///Thin wrapper over cpSpace
///It graps ownership of any shape, body or constraint
class Space {
public:
    Space();
    ~Space();
    cpSpace *native() const;
    void add_shape(cpShape *);
    void add_body(cpBody *);
    void add_constraint(cpConstraint *);
    void clear();

    cpBody *static_body() const;
private:
    std::vector<cpShape*> shapes_;
    std::vector<cpBody*> bodies_;
    std::vector<cpConstraint*> constraints_;

    cpSpace *impl_ = nullptr;

    ///After first transaction space becomes readonly - no add operation permitted, flag reset on clear
    mutable bool readonly_ = false;
};

} // namespace cp