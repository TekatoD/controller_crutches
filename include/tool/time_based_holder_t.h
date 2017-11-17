/// \autor arssivka
/// \date 11/15/17

#pragma once


#include <utility>
#include "rate_t.h"

namespace drwn {
    template <class T>
    class time_based_holder_t {
    public:
        using type = T;
        
        explicit time_based_holder_t(const steady_rate_t& rate, T item = type())
                : m_item(std::move(item)), m_rate(rate) {}
        
        void set_item_and_update(type item) {
            m_item = std::move(item);
            m_rate.update();
        }
        
        void set_item_without_update(type item) {
            m_item = std::move(item);
        }

        const type& get_item() const {
            return m_item;
        }

        bool is_valid() const {
            return !m_rate.is_passed();
        }

    private:
        type m_item{};
        steady_rate_t m_rate;
    };
}



