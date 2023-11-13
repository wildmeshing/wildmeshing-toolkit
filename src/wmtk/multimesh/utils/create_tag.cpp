#include "create_tag.hpp"

namespace wmtk::multimesh::utils {
bool is_root(std::vector<int>& parents, int vid)
{
    return parents[vid] == vid;
}

int get_root(std::vector<int>& parents, int vid)
{
    if (parents[vid] == -1) {
        return -1;
    }
    while (!is_root(parents, vid)) {
        vid = parents[vid];
    }
    return vid;
}

void set_root(std::vector<int>& parents, int vid, int root)
{
    while (!is_root(parents, vid)) {
        int tmp = parents[vid];
        parents[vid] = root;
        vid = tmp;
    }
    parents[vid] = root;
}

void set_union(std::vector<int>& parents, int vid1, int vid2)
{
    int root1 = get_root(parents, vid1);
    int root2 = get_root(parents, vid2);
    int root = std::min(root1, root2);
    if (root == -1) {
        root = std::min(vid1, vid2);
    }
    set_root(parents, vid1, root);
    set_root(parents, vid2, root);
}

void union_find(std::set<int>& critical_points, std::vector<int>& parents, int vid1, int vid2)
{
    if (critical_points.find(vid1) == critical_points.end() &&
        critical_points.find(vid2) == critical_points.end()) {
        return;
    }
    if (critical_points.find(vid1) == critical_points.end()) {
        set_root(parents, vid1, vid2);
    } else if (critical_points.find(vid2) == critical_points.end()) {
        set_root(parents, vid2, vid1);
    } else {
        set_union(parents, vid1, vid2);
    }
}

} // namespace wmtk::multimesh::utils