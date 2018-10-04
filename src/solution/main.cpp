#include "logic/strategy.h"
#include "simulation/simulator.h"
#include "common/logger.h"
#include "common/json.h"
#include "common/RewindClient.h"
#include "structures.h"

#include <vector>
#include <cstdio>

unsigned int RANDOM_SEED = 42;

void local_dump(std::string_view line) {
#ifdef LOCAL_RUN
    static FILE *local_file = fopen("latest-game.txt", "w");
    fputs(line.data(), local_file);
#endif
}

int main(int argc, char *argv[]) {
#ifdef ENABLE_LOG
    loguru::g_preamble_thread = false;
    loguru::g_preamble_date = false;
    loguru::g_stderr_verbosity = 1;

    loguru::init(argc, argv);
    loguru::add_file("strategy.log", loguru::Truncate, 6);
#endif

    FILE *inp_stream = stdin;
    if (argc > 1) {
        const char *replay = argv[1];
        inp_stream = fopen(replay, "r");
        if (!inp_stream) {
            LOG_FATAL("Cannot open local file %s", replay);
            return -1;
        }
        LOG_INFO("Replaying file %s", replay);
    }

    const bool is_replay = inp_stream != stdin;

    char buf[65536];
    Strategy agent;
    bool is_exit_requested = false;
    for (int turn = 0; !is_exit_requested; ++turn) {
        fgets(buf, sizeof(buf), inp_stream);
        if (!is_replay) {
            local_dump(buf);
        }

        LOG_DEBUG("Overall iteration %d", turn);

        const auto j = nlohmann::json::parse(buf);
        LOG_V7("Input json: %s", j.dump().c_str());

        const auto type = j["type"].get<std::string>();

        if (type == "new_match") {
            LOG_DEBUG("New match started");
            agent.next_match(j["params"].get<Game>());
        } else if (type == "tick") {
            Action decision = agent.move(j["params"].get<World>());
            if (!is_replay) {
                puts(action_to_command(decision, agent.debug_string()).c_str());
            }
        } else {
            is_exit_requested = true;
        }

        fflush(stdout);
    }

    if (is_replay) {
        fclose(inp_stream);
    }

    return 0;
}