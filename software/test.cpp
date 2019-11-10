#include <iostream>

using namespace std;

int main(){
    for(int i = 0; i < 100000; i++){
        cout << "\r[";
        int n;
        for(n = 0; n < i / 2000; n++){
            cout << '|';
        }
        for(int j = n; j < 49; j++){
            cout << " ";
        }
        cout << "]" << flush;
    }
}