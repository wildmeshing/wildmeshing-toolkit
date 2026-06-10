// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Jeremie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Jeremie Dumas on 09/04/18.
//

#include "Common.h"

#include <algorithm>
#include <fstream>
#include <wmtk/utils/Logger.hpp>

#include "Args.h"
#include "State.h"

namespace wmtk::components::tetwild::orig {

bool isHaveCommonEle(const std::unordered_set<int>& v1, const std::unordered_set<int>& v2)
{
#if 0
    for (auto it = v1.begin(); it != v1.end(); it++)
        if(std::find(v2.begin(), v2.end(), *it)!=v2.end())
            return true;
#else
    if (v2.size() < v1.size()) {
        return isHaveCommonEle(v2, v1);
    }
    for (int x : v1) {
        if (v2.count(x)) {
            return true;
        }
    }
#endif
    return false;
}

void setIntersection(
    const std::unordered_set<int>& s1,
    const std::unordered_set<int>& s2,
    std::unordered_set<int>& s)
{
#if 0
    std::unordered_set<int> s_tmp;
    std::vector<int> v1, v2;
    v1.reserve(s1.size());
    for (auto it = s1.begin(); it != s1.end(); it++)
        v1.push_back(*it);
    v2.reserve(s2.size());
    for (auto it = s2.begin(); it != s2.end(); it++)
        v2.push_back(*it);
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(s_tmp, s_tmp.end()));
    s = s_tmp;
#else
    if (s2.size() < s1.size()) {
        setIntersection(s2, s1, s);
        return;
    }
    s.clear();
    s.reserve(std::min(s1.size(), s2.size()));
    for (int x : s1) {
        if (s2.count(x)) {
            s.insert(x);
        }
    }
#endif

    //    s.clear();
    //    s.reserve(std::min(s1.size(), s2.size()));
    //    std::unordered_set<int> s_tmp = s2;
    //    int size = s_tmp.size();
    //    for(int ele:s1){
    //        s_tmp.insert(ele);
    //        if(s_tmp.size()>size){
    //            size = s_tmp.size();
    //        } else
    //            s.insert(ele);
    //    }
}

void setIntersection(
    const std::unordered_set<int>& s1,
    const std::unordered_set<int>& s2,
    std::vector<int>& s)
{
//    s.clear();
//    s.reserve(std::min(s1.size(), s2.size()));
//    std::unordered_set<int> s_tmp = s2;
//    int size = s_tmp.size();
//    for(int ele:s1){
//        s_tmp.insert(ele);
//        if(s_tmp.size()>size){
//            size = s_tmp.size();
//        } else
//            s.push_back(ele);
//    }
#if 0
    std::vector<int> v1, v2;
    v1.reserve(s1.size());
    for(auto it=s1.begin();it!=s1.end();it++)
        v1.push_back(*it);
    v2.reserve(s2.size());
    for(auto it=s2.begin();it!=s2.end();it++)
        v2.push_back(*it);
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(s));
#else
    if (s2.size() < s1.size()) {
        setIntersection(s2, s1, s);
        return;
    }
    s.clear();
    s.reserve(std::min(s1.size(), s2.size()));
    for (int x : s1) {
        if (s2.count(x)) {
            s.push_back(x);
        }
    }
    std::sort(s.begin(), s.end());
#endif
}

} // namespace wmtk::components::tetwild::orig
