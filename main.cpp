//
// Created by Yixin Hu on 10/17/21.
//

#include "src/TetMesh.h"
#include "src/TetWild.h"

int main(int argc, char** argv){
    using namespace wmtk;

    Parameters params;
    Envelope envelope;
    TetWild tetwild(params, envelope);

    tetwild.test();

    return 0;
}