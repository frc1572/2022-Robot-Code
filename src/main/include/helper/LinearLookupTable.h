#pragma once

#include <initializer_list>
#include <iterator>
#include <map>

template <typename Key, typename T> class LinearLookupTable
{
public:
    LinearLookupTable(std::initializer_list<typename std::map<Key, T>::value_type> data) : m_data(data)
    {
    }

    T Lookup(Key key)
    {
        auto upperPoint = m_data.lower_bound(key);
        if (upperPoint == m_data.begin())
        {
            return upperPoint->second;
        }
        if (upperPoint == m_data.end())
        {
            return m_data.rbegin()->second;
        }
        auto lowerPoint = std::prev(upperPoint);
        auto t = (key - lowerPoint->first) / (upperPoint->first - lowerPoint->first);
        return lowerPoint->second + (upperPoint->second - lowerPoint->second) * t;
    }

private:
    std::map<Key, T> m_data;
};