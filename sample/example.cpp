// tmp.cpp
#include <iostream>
#include <test.hpp>
using namespace std;

namespace impl {
    int addition(int x, int y) {
        return x + y;
    }

    void f() {
        addition(2, 3);
    }
}

int addition (int a, int b) {
  int r;
  r=a+b;
  return r;
}


int main () {
  int z, q;
  //swap(x,q);
  funTest();
  z = addition (5,3);
  q = addition (5,5);
  cout << "The first result is " << z;
  cout << "The second result is " << q;
}