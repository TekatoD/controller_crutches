/// \autor arssivka
/// \date 10/31/17

#pragma once


namespace drwn {
    class request_t;

    class request_handler_t {
    public:
        virtual void process(request_t&) = 0;
    };
}



