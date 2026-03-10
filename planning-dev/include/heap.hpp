#ifndef HEAP_H
#define HEAP_H

#include <functional>
#include <unordered_map>
#include <vector>

template<typename T>
class Heap {
public:
    Heap(std::function<bool(const T&, const T&)> compare);

    bool push(T item, int unique_id);

    // get top item of heap
    T top_item();

    // get top unique item id of heap
    int top_item_id();
    
    // removes top of heap
    void pop();

    unsigned int get_size() {
        return size;
    }

    ~Heap();

private:
    std::vector<T> heap;
    
    std::vector<int> hid2id;
    std::unordered_map<int, unsigned int> id2hid; 

    unsigned int size;

    // compare: a is before b in the heap
    // return true if swap a and b, return false if no swap
    std::function<bool(const T& a, const T& b)> compare;

    void swap(unsigned int hida, unsigned int hidb);

    // Move item up (because its priority increases)
    void up(int unique_id);
};


template<typename T>
Heap<T>::Heap(std::function<bool(const T&, const T&)> compare) {
    this->size = 0;
    this->compare = compare;
}


template<typename T>
bool Heap<T>::push(T item, int unique_id) {
    if (this->id2hid.find(unique_id) != this->id2hid.end()) {
        //std::cout << "heap push unique id exists! " << unique_id << std::endl;
        //std::cout << "heap push id2hid! " << this->id2hid[unique_id] << std::endl;
        //std::cout << "heap push size! " << this->size << std::endl;
        //std::cout << "compare! " << unique_id << std::endl;
        //std::cout << this->compare(this->heap[this->id2hid[unique_id]], item) << std::endl;
        if (this->compare(this->heap[this->id2hid[unique_id]], item)) {
            this->heap[this->id2hid[unique_id]] = std::move(item);
            up(unique_id);
            return true;
        }
        return false;
    }

    //std::cout << "heap push new unique id! " << unique_id << std::endl;

    if (size < this->heap.size()) {
        this->heap[size] = std::move(item);
        this->hid2id[size] = unique_id;
    } else {
        this->heap.push_back(std::move(item));
        this->hid2id.push_back(unique_id);
    }

    this->id2hid[unique_id] = size;

    unsigned int cindex = this->size;
    unsigned int pindex = (cindex - 1) / 2;
    
    while (cindex != 0 && this->compare(this->heap[pindex], this->heap[cindex])) {
        this->swap(cindex, pindex);
        cindex = pindex;
        pindex = (cindex - 1) / 2;
    }

    this->size += 1;
    return true;
}

template<typename T>
void Heap<T>::pop() {
    if (this->size == 0) return;

    //std::cout << "pop id2hid " << this->hid2id[0] << std::endl;


    this->id2hid.erase(this->hid2id[0]);
    this->heap[0] = std::move(this->heap[this->size - 1]);
    this->hid2id[0] = this->hid2id[this->size - 1];
    this->id2hid[this->hid2id[this->size - 1]] = 0;

    this->size -= 1;

    unsigned int pindex = 0;
    unsigned int lindex = 0;
    unsigned int rindex = 0;
    unsigned int cindex = 0;

    while (2 * pindex + 1 < this->size) {
        lindex = 2 * pindex + 1;
        rindex = 2 * pindex + 2;

        cindex = lindex;
        if (rindex < this->size && this->compare(this->heap[lindex], this->heap[rindex])) {
            cindex = rindex;
        }
        if (this->compare(this->heap[pindex], this->heap[cindex])) {
            this->swap(pindex, cindex);
            pindex = cindex;
        } else {
            break;
        }
    }
}

template<typename T>
T Heap<T>::top_item() {
    return std::move(this->heap[0]);
}

template<typename T>
int Heap<T>::top_item_id() {
    return this->hid2id[0];
}

template<typename T>
void Heap<T>::up(int unique_id) {
    unsigned int cindex = id2hid[unique_id];
    unsigned int pindex = (cindex - 1) / 2;
    while (cindex != 0 && this->compare(this->heap[pindex], this->heap[cindex])) {
        this->swap(pindex, cindex);
        cindex = pindex;
        pindex = (cindex - 1) / 2;
    }
}

template<typename T>
void Heap<T>::swap(unsigned int hida, unsigned int hidb) {
    T a = std::move(this->heap[hida]);
    this->heap[hida] = std::move(this->heap[hidb]);
    this->heap[hidb] = std::move(a);

    int uqida = this->hid2id[hida];
    this->hid2id[hida] = this->hid2id[hidb];
    this->hid2id[hidb] = uqida;

    this->id2hid[this->hid2id[hidb]] = hidb;
    this->id2hid[this->hid2id[hida]] = hida;
}

template<typename T>
Heap<T>::~Heap() {
    this->heap.clear();
}

#endif