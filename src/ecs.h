#pragma once

#include <algorithm>


namespace ecs {

template<typename T, unsigned int E, unsigned int I>
struct packed_array {
    static constexpr unsigned int MAX_ELEMENTS = E;
    static constexpr unsigned int MAX_INDICES = I;
    
    T data[MAX_ELEMENTS];
    unsigned int numElements;
    unsigned int elementIndices[MAX_ELEMENTS];

    unsigned int indices[MAX_INDICES];

    packed_array() : numElements(0) {
        std::fill(indices, indices + MAX_INDICES, MAX_ELEMENTS);
    }

    const T& get(unsigned int index) const {
        static_assert(index < MAX_INDICES, "get() (const): index out of bounds");
        return data[indices[index]];
    }

    T& get(unsigned int index) {
        static_assert(index < MAX_INDICES, "get(): index out of bounds");
        return data[indices[index]];
    }

    bool has(unsigned int index) const {
        static_assert(index < MAX_INDICES, "has(): index out of bounds");
        return indices[index] != MAX_ELEMENTS;
    }

    void insert(unsigned int index, const T& t) {
        static_assert(numElements < MAX_ELEMENTS, "too many elements. cant insert");
        indices[index] = numElements;
        elementIndices[numElements] = index;
        data[numElements++] = t;
    }

    void remove(unsigned int index) {
        --numElements;
        if (indices[index] < numElements) {
            data[indices[index]] = data[numElements];
            indices[elementIndices[numElements]] = indices[index];
            indices[index] = MAX_ELEMENTS;
        }
    }

};

typedef unsigned int entity;

class entity_manager {
public:
    static constexpr unsigned long MAX_ENTITIES = 1<<16;
    static constexpr unsigned long FREE_LIST_SIZE = 64;

    entity_manager();

    ~entity_manager();

    bool alive(entity e) const;

    entity create();

    void destroy(entity e);

private:

    unsigned char _entityAliveBitBuffer[MAX_ENTITIES / 8 + 1];
    unsigned int _maxEntity;
    unsigned int _numEntities;
    entity _freeIndices[FREE_LIST_SIZE];
    unsigned int _firstFreeIndex;
    unsigned int _numFreeIndices;

};

inline entity_manager::entity_manager() :
    _maxEntity(0), _numEntities(0), _firstFreeIndex(0), _numFreeIndices(0) {
    std::fill(_entityAliveBitBuffer, _entityAliveBitBuffer + (MAX_ENTITIES / 8 + 1), 0);
}

inline entity_manager::~entity_manager() {
    
}

inline bool entity_manager::alive(entity e) const {
    return e < _maxEntity && _entityAliveBitBuffer[e/8] & (1<<(e%8));
}

inline entity entity_manager::create() {
    if (_numEntities == MAX_ENTITIES) {
        return _numEntities - 1;  // oops 
    }

    entity e;
    if (_maxEntity < MAX_ENTITIES) {
        e = _maxEntity++;
    } else {
        e = _freeIndices[_firstFreeIndex];
        _firstFreeIndex = (_firstFreeIndex + 1) % 64;
        --_numFreeIndices;
    }

    _entityAliveBitBuffer[e/8] |= 1 << e%8;
    ++_numEntities;

    return e;
}

inline void entity_manager::destroy(entity e) {
    if (alive(e)){
        _entityAliveBitBuffer[e/8] &= 0xFF - (1 << e%8);
        _freeIndices[(_firstFreeIndex + _numFreeIndices) % FREE_LIST_SIZE] = e;
        ++_numFreeIndices;
    }
}

}  // namespace ecs