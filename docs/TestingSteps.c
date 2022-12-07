#include <stdio.h>
// uint8_t position = 0;
// uint8_t target = 0;

// uint8_t error= 0;
// uint8_t preverror = 0;

// uint8_t CurrDel = 0;


void main(){
    int Dir = 0;
    int CurError = 0;
    Dir = (CurError > 0) - (CurError < 0);
    printf("dir= %d\n",Dir);


}

// int step(){
//     error = position-target;
//     dir = error>0 - error<0;
//     if(error != 0){
//         if(stepflag){
            
//             PORTB=
//             reset timer
//             CURRPos+dir
//             position= position+dir;
//             if(CurPos>4){
//                 CurPos = 0
//             }
//             if(CurPos>3){
//                 CurPos = 4
//             }
//             if
//         }
//     }
// }