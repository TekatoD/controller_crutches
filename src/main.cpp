#include <csignal>
#include <libgen.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>
#include <log/ColouredSink.h>
#include <log/Logger.h>

#include <RobotApplication.h>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

void change_current_dir() {
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
        if (chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void init_logger() {
    using coloured_sink = sinks::synchronous_sink<Robot::ColouredSink>;
    auto sink = boost::make_shared<coloured_sink>();
    sink->set_formatter(expr::stream
                                << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%H:%M:%S.%f")
                                << " [" << logging::trivial::severity
                                << "] " << expr::smessage);
    logging::core::get()->set_filter(
#ifdef DEBUG
            logging::trivial::severity >= logging::trivial::debug
#else
            logging::trivial::severity >= logging::trivial::info
#endif
    );
    logging::core::get()->add_sink(std::move(sink));
}


void sighandler(int /*sig*/) {
    Robot::RobotApplication::GetInstance()->Stop();
}


int main(int argc, const char** argv) {
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();
    init_logger();

    Robot::RobotApplication::GetInstance()->ParseArguments(argc, argv);
    return Robot::RobotApplication::GetInstance()->Exec();
}
