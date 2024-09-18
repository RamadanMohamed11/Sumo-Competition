#include <iostream>
#include <bitset>
#include <iomanip>

using namespace std;

enum IRStates
{
    _NONE,
    _BL,
    _BR,
    _BLBR,
    _FR,
    _FRBL,
    _FRBR,
    _FRBRBL,
    _FL,
    _FLBL,
    _FLBR,
    _FLBRBL,
    _FLFR,
    _FLFRBL,
    _FLFRBR,
    _FLFRBRBL,
};

// "_NONE"
//     "_BL"
//     "_BR"
//     "_BLBR"
//     "_FR"
//     "_FRBL"
//     "_FRBR"
//     "_FRBRBL"
//     "_FL"
//     "_FLBL"
//     "_FLBR"
//     "_FLBRBL"
//     "_FLFR"
//     "_FLFRBL"
//     "_FLFRBR"
//     "_FLFRBRBL"

char checkIRSensors()
{
    char result = 0x00;
    // result |= (IRFrontLeft.detectWhiteLine() << 3) | (IRFrontLeft.detectWhiteLine() << 2) | (IRFrontLeft.detectWhiteLine() << 1) | (IRFrontLeft.detectWhiteLine() << 0);

    result |= (true << 3) | (true << 2) | (true << 1) | (true << 0);
    return result;
}

// int main()
// {

//     char result = checkIRSensors();

//     switch (result)
//     {

//     case IRStates::_NONE:
//         cout << "_NONE";
//         break;
//     case IRStates::_BL:
//         cout << "_BL";
//         break;
//     case IRStates::_BR:
//         cout << "_BR";
//         break;
//     case IRStates::_BLBR:
//         cout << "_BLBR";
//         break;
//     case IRStates::_FR:
//         cout << "_FR";
//         break;
//     case IRStates::_FRBL:
//         cout << "_FRBL";
//         break;
//     case IRStates::_FRBR:
//         cout << "_FRBR";
//         break;
//     case IRStates::_FRBRBL:
//         cout << "_FRBRBL";
//         break;
//     case IRStates::_FL:
//         cout << "_FL";
//         break;
//     case IRStates::_FLBL:
//         cout << "_FLBL";
//         break;
//     case IRStates::_FLBR:
//         cout << "_FLBR";
//         break;
//     case IRStates::_FLBRBL:
//         cout << "_FLBRBL";
//         break;
//     case IRStates::_FLFR:
//         cout << "_FLFR";
//         break;
//     case IRStates::_FLFRBL:
//         cout << "_FLFRBL";
//         break;
//     case IRStates::_FLFRBR:
//         cout << "_FLFRBR";
//         break;
//     case IRStates::_FLFRBRBL:
//         cout << "_FLFRBRBL";
//         break;
//     }

//     return 0;
// }

int main()
{

    char result = 0x00;
    bool sensors[4] = {false, true, false, true};
    for (int i = 0; i < 4; i++)
        result |= (sensors[i] << (4 - 1 - i));

    cout << setw(4) << (int)result << " " << bitset<4>(result);

    return 0;
}