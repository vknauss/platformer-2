#pragma once

#include <limits>
#include <vector>


template<typename mapped_t, typename id_t = unsigned int>
class mapped_storage {

    std::vector<mapped_t> _data;
    std::vector<id_t> _ids;

public:

    static constexpr id_t id_null = std::numeric_limits<id_t>::max();

    void resize(size_t size);

    void assign(const std::vector<mapped_t>& data);

    void push_back(const mapped_t& element);

    template<typename compare_t>
    void sort_ids(const compare_t& compare);

    template<typename query_t, typename compare_t>
    id_t find_id(const query_t& query, const compare_t& compare) const;

    const std::vector<mapped_t>& data() const { return _data; }
    // std::vector<mapped_t>& data() { return _data; }
    
    const std::vector<id_t>& ids() const { return _ids; }
    // std::vector<id_t>& ids() { return _ids; }

};


template<typename mapped_t, typename id_t>
void mapped_storage<mapped_t, id_t>::resize(size_t size) {
    auto oldsize = _data.size();
    _data.resize(size);
    _ids.resize(size);
    for (auto i = oldsize; i < size; ++i) {
        _ids[i] = i;
    }
}

template<typename mapped_t, typename id_t>
void mapped_storage<mapped_t, id_t>::assign(const std::vector<mapped_t>& data) {
    auto oldsize = _data.size();
    _data.assign(data);
    _ids.resize(data.size());
    for (auto i = oldsize; i < _ids.size(); ++i) {
        _ids[i] = i;
    }
}

template<typename mapped_t, typename id_t>
void mapped_storage<mapped_t, id_t>::push_back(const mapped_t &element) {
    _data.push_back(element);
    _ids.push_back(_ids.size());
}


template<typename mapped_t, typename id_t> template<typename compare_t>
void mapped_storage<mapped_t, id_t>::sort_ids(const compare_t& compare) {
    for (auto i = 0u; i < _ids.size(); ++i) {
        id_t id0 = _ids[i];
        // iterate backwards, swap ids if mapped values are out of order, break otherwise
        for (auto j = 0u; j < i; ++j) {
            id_t id1 = _ids[i-j-1];
            if (compare(_data[id0], _data[id1]) < 0) {
                _ids[i-j-1] = id0;
                _ids[i-j] = id1;
                continue;
            }
            break;
        }
    }
}

template<typename mapped_t, typename id_t> template<typename query_t, typename compare_t>
id_t mapped_storage<mapped_t, id_t>::find_id(const query_t& query, const compare_t& compare) const {
    id_t pos = _ids.size() / 2;
    id_t min = 0;
    id_t end = _ids.size();
    // binary search for pair in sorted list of collision_pairs
    // the runtime of this search for all pairs is O(bpairs.size() * log2(_pairs.size()))
    // if bpairs were sorted, we could run through both arrays in linear time, but of course
    // there's the cost of sorting. maybe revisit.
    while (end - min > 0) {
        auto c = compare(query, _data[_ids[pos]]);
        if (c < 0) {
            // search left
            end = pos;
        } else if (c > 0) {
            // search right
            min = pos + 1;
        } else {
            // found
            return pos;
        }
        pos = min + (end - min) / 2;
    }
    return id_null;
}