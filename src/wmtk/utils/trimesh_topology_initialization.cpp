#include <wmtk/trimesh_topology_initialization.h>
namespace wmtk {
namespace {

void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF)
{
    // Make sure there are 3 columns
    assert(F.cols()==3);

    char iv0 = 0;
    char iv1 = 1;
    char it = 2;
    char ii = 3;

    std::vector<std::vector<long>> TTT;

    long vertex_count = F.coeffWise().max();
    
    // Build a table for finding Faces and populate the corresponding
    // topology relations
    {
        TTT.resize(T.rows()*4);
        for (int t = 0; t < T.rows(); ++t) {
            for (int i = 0; i < 3; ++i) {
                // v1 v2 v3 f ei
                long x = std::static_cast<long>(auto_2d_edge[i][0]);
                long y = std::static_cast<long>(auto_2d_edge[i][1]);
                if (x > y) swap(x, y);
                
                std::vector<long> r(4);
                r[iv0] = x;
                r[iv1] = y;
                r[it] = t;
                r[ii] = i;
                TTT[t*3+i] = r;
            }
        }
        std::sort(TTT.begin(), TTT.end());

        // VF
        VF.resize(vertex_count,1);
        for (int i = 0; i < F.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                VT[F[i,j]] = i;
            }
        }

        // Compute FE, FF, EF
        FE.resize(T.rows(), 3);
        FF.resize(T.rows(), 3);
        vector<long> EF_temp;

        // iterate over TTT to find faces
        // for every entry check if the next is the same, and update the connectivity accordingly

        for (int i = 0; i < TTT.size(); ++i) {
            if ((i==(TTT.size()-1)) || (TTT[i][0] != TTT[i+1][0]) || (TTT[i][1] != TTT[i+1][1]))
            {
                // If the next tuple is empty, then this is a boundary edge
                EF_temp.push_back(TTT[i][it]);
                
                TT(TTT[i][it],TTT[i][ii]) = -1;
                TF(TTT[i][it],TTT[i][ii]) = EF_temp.size()-1;
            }
            else
            {
                // this is an internal edge, update both sides           
                EF_temp.push_back(TTT[i][it]);
                
                TT(TTT[i][it],TTT[i][ii]) = TTT[i+1][it];
                TF(TTT[i][it],TTT[i][ii]) = EF_temp.size()-1;
                
                TT(TTT[i+1][it],TTT[i+1][ii]) = TTT[i][it];
                TF(TTT[i+1][it],TTT[i+1][ii]) = EF_temp.size()-1;

                ++i; // skip the other entry
            }
        }

        // copy EF
        EF.resize(EF_temp.size());
        for (long i=0; i<EF_temp.size();++i)
            EF(i) = EF_temp(i);
    }
}

} // namespace wmtk
