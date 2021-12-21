//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <algorithm>

namespace wmtk
{
	template <class T>
	inline std::vector<T> unsorted_set_intersection(const std::vector<T> &v1, const std::vector<T> &v2)
	{
		auto s1 = v1;
		auto s2 = v2;
		std::sort(s1.begin(), s1.end());
		std::sort(s2.begin(), s2.end());
		std::vector<T> v;
		std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v));
		return v;
	}

    template <class T>
    inline std::vector<T> set_intersection(const std::vector<T> &v1, const std::vector<T> &v2)
    {
        std::vector<T> v;
        std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v));
        return v;
    }

	template <class T>
	inline void vector_unique(std::vector<T> &v)
	{
		std::sort(v.begin(), v.end());
		v.erase(std::unique(v.begin(), v.end()), v.end());
	}

    template <class T>
    inline void vector_print(std::vector<T> &v)
    {
        for(auto& e: v)
            std::cout<<e<<" ";
        std::cout<<std::endl;
    }

    template <class T>
    inline void vector_sort(std::vector<T> &v)
    {
        std::sort(v.begin(), v.end());
    }

	template <class T>
	inline bool vector_erase(std::vector<T> &v, const T &t)
	{
		auto it = std::find(v.begin(), v.end(), t);
		if (it == v.end())
			return false;
		v.erase(it);
		return true;
	}
} // namespace wmtk
