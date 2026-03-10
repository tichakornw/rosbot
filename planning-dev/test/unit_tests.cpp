#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include "../include/heap.hpp"
#include "../include/multicost.hpp"
#include "../include/multicost_array.hpp"
#include "../include/iterated_dijkstra_propagation.hpp"
#include "../include/multicost_compute.hpp"

bool print_test = false;
int num_errors = 0;

void test(int expect, int received, std::string fname) {
    if (expect != received) {
        std::cout << "ERROR FROM [" << fname << "] !!! " << "expected: [" << expect << "] received: [" << received << "]\n"; 
        num_errors += 1;
    } else if (print_test) {
        std::cout << "SUCCESSFUL TEST [" << fname << "] " << "expected: [" << expect << "] received: [" << received << "]\n"; 
    }
}


void test_multicost_comp_op() {
    std::string fname = "TEST MULTICOST COMP AND OP";
    std::cout << "> " << fname << std::endl;

    auto multicost_prop = PolyMulticostProps<int, int> (
        0, 0, 
        [](int a, int b){return a < b ? -1 : ((a > b) ? 1 : 0) ;},
        [](int a, int b){return a > b ? -1 : ((a < b) ? 1 : 0);},
        [](int a, int b){return a + b;},
        [](int a, int b){return std::max(a, b);}
    );

    auto val1 = std::tuple<int, int> (
        20, 40
    );

    auto val2 = std::tuple<int, int> (
        1, 10
    );

    auto val3 = multicost_prop.op(val1, val2);

    auto raw_v = val3;
    test(21, std::get<0>(raw_v), fname);
    test(40, std::get<1>(raw_v), fname);

    test(1, multicost_prop.compare(val1, val2), fname);
    test(-1, multicost_prop.compare(val2, val1), fname);
    test(0, multicost_prop.compare(val1, val1), fname);
}

void test_multicost_array() {
    std::string fname = "TEST MULTICOST ARRAY";
    std::cout << "> " << fname << std::endl;

    auto multicost_prop = PolyMulticostProps<int, int> (
        0, 0, 
        [](int a, int b){return a < b ? -1 : ((a > b) ? 1 : 0) ;},
        [](int a, int b){return a > b ? -1 : ((a < b) ? 1 : 0);},
        [](int a, int b){return a + b;},
        [](int a, int b){return std::max(a, b);}
    );
    
    auto multicost_array = 
        std::make_shared<PolyMulticostArray<int, int>>(multicost_prop);
    
    auto index_val1 = multicost_array->make_multicost(std::tuple(10, 5));
    test(0, index_val1->get_id(), fname);

    auto index_val2 = multicost_array->make_multicost(std::tuple(2, 20));
    test(1, index_val2->get_id(), fname);

    auto index_val3 = multicost_array->make_multicost(std::tuple(30, 10));
    test(2, index_val3->get_id(), fname);

    auto index_val4 = multicost_array->op(index_val1, index_val2);
    test(3, index_val4->get_id(), fname);

    test(4, multicost_array->num_values(), "num values");
    test(4, multicost_array->allocated_size(), "allocate size");

    // DO NOT ACTUALLY DO THIS, THIS IS ONLY FOR TESTING
    auto *raw = index_val3.release();
    delete raw;

    test(3, multicost_array->num_values(), "num values after release");
    test(4, multicost_array->allocated_size(), "allocate size after release");

    auto index_val5 = multicost_array->op(index_val1, index_val2);
    auto val5 = multicost_array->get_values(index_val5);
    test(12, std::get<0>(val5), "get value 1 from val5");
    test(20, std::get<1>(val5), "get value 2 from val5");
    test(2, index_val5->get_id(), "id of val5");
    test(4, multicost_array->num_values(), "val5 num values after op");
    test(4, multicost_array->allocated_size(), "val5 allocate size after op");

    auto index_val6 = multicost_array->op(index_val5, index_val5);
    auto val6 = multicost_array->get_values(index_val6);
    test(24, std::get<0>(val6), "get value 1 from val6");
    test(20, std::get<1>(val6), "get value 2 from val6");
    test(4, index_val6->get_id(), "id of val6");
    test(5, multicost_array->num_values(), "val6 num values after op");
    test(5, multicost_array->allocated_size(), "val6 allocate size after op");



}

void test_heap_int() {
    std::string fname = "HEAP INIT TEST";
    std::cout << "> " << fname << std::endl;
    auto heap = Heap<int>([](int a, int b) {return b < a;});

    for (int i = 10; i > 0; --i) {
        heap.push(i, i);
    }

    for (int i = 1; i < 11; ++i) {
        test(i, heap.top_item(), fname);
        heap.pop();
    }

    for (int i = 10; i > 0; --i) {
        heap.push(i, i);
    }

    heap.pop();
    test(2, heap.top_item(), fname);

    heap.push(0, 5);
    test(0, heap.top_item(), fname);

}


int main(int argc, char* argv[]) {
    if (argc > 1) {
        std::string sp = argv[1];
        if (sp.compare("1") == 0) {
            print_test = true;
        }
    }

    std::cout << "-- BEGIN TESTS --" << std::endl;
    

    //test_multicost_comp_op();
    //test_multicost_array();
    //test_heap_int();
    //test_pathfind_dijkstra();
    //test_pathfind_iterated();

    return 0;
}