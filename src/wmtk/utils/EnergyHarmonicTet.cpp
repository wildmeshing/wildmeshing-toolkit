#include "EnergyHarmonicTet.hpp"
#include <limits>
// Generated with following snippet with some text replacement.
// Also modified for infinity evaluation
// ```python
// from sympy.printing import ccode
// from sympy import symbols, cse, numbered_symbols, Matrix, det,diff
// a00,a01,a02,a10,a11,a12,a20,a21,a22, a30,a31,a32 = symbols(','.join([f'T[{i}]' for i in
// range(12)])) pts = Matrix([[a00,a01,a02],[a10,a11,a12],[a20,a21,a22],[a30,a31,a32]]) vol =
// det(Matrix([pts.row(1) - pts.row(0), pts.row(2) - pts.row(0),pts.row(3) - pts.row(0)])) def
// area2(i,j,k):
//     cr = (pts.row(i) - pts.row(j)).cross(pts.row(i) - pts.row(k))
//     squ_area = cr.dot(cr) / 4
//     return squ_area
// energy = (area2(0,1,2) + area2(0,1,3) + area2(0,2,3) + area2(1,2,3))/vol
// # Following from https://stackoverflow.com/a/43452434/5742040
// CSE_results = cse(energy,numbered_symbols("helper_"))
//
// # CSE_results = cse([diff(energy, a00), diff(energy, a01), diff(energy,
// a02)],numbered_symbols("helper_"))
//
// with open("snippet_diff.c", "w") as output:
//     for helper in CSE_results[0]:
//         output.write("double ")
//         output.write(ccode(helper[1],helper[0]))
//         output.write("\n")
//
//     for i,result in enumerate(CSE_results[1]):
//         output.write(ccode(result,"auto result_%i"%i))
//         output.write("\n")
// ```
using std::pow;
double wmtk::harmonic_tet_energy(const std::array<double, 12>& T)
{
    double helper0 = T[0] * T[10];
    double helper1 = T[0] * T[11];
    double helper2 = T[5] * T[7];
    double helper3 = T[10] * T[2];
    double helper4 = T[5] * T[6];
    double helper5 = T[11] * T[1];
    double helper6 = T[3] * T[7];
    double helper7 = T[3] * T[8];
    double helper8 = T[1] * T[9];
    double helper9 = T[4] * T[6];
    double helper10 = T[2] * T[9];
    double helper11 = T[4] * T[8];
    double helper12 = T[0] - T[3];
    double helper13 = -T[10];
    double helper14 = T[1] + helper13;
    double helper15 = -T[9];
    double helper16 = T[0] + helper15;
    double helper17 = T[1] - T[4];
    double helper18 = T[2] - T[5];
    double helper19 = -T[11];
    double helper20 = T[2] + helper19;
    double helper21 = -T[7];
    double helper22 = T[1] + helper21;
    double helper23 = -T[6];
    double helper24 = T[0] + helper23;
    double helper25 = -T[8];
    double helper26 = T[2] + helper25;
    double helper27 = T[4] + helper13;
    double helper28 = T[3] + helper23;
    double helper29 = T[3] + helper15;
    double helper30 = T[4] + helper21;
    double helper31 = T[5] + helper19;
    double helper32 = T[5] + helper25;
    auto denom = (-T[0] * helper11 + T[0] * helper2 + T[10] * helper4 - T[10] * helper7 + T[11] * helper6 -
         T[11] * helper9 - T[1] * helper4 + T[1] * helper7 - T[2] * helper6 + T[2] * helper9 +
         T[3] * helper3 - T[3] * helper5 + T[4] * helper1 - T[4] * helper10 - T[5] * helper0 +
         T[5] * helper8 - T[6] * helper3 + T[6] * helper5 - T[7] * helper1 + T[7] * helper10 +
         T[8] * helper0 - T[8] * helper8 + T[9] * helper11 - T[9] * helper2);
    if (denom < 0) return std::numeric_limits<double>::infinity();
    auto result_0 =
        ((1.0 / 4.0) * pow(helper12 * helper14 - helper16 * helper17, 2) +
         (1.0 / 4.0) * pow(-helper12 * helper20 + helper16 * helper18, 2) +
         (1.0 / 4.0) * pow(helper12 * helper22 - helper17 * helper24, 2) +
         (1.0 / 4.0) * pow(-helper12 * helper26 + helper18 * helper24, 2) +
         (1.0 / 4.0) * pow(-helper14 * helper18 + helper17 * helper20, 2) +
         (1.0 / 4.0) * pow(helper14 * helper24 - helper16 * helper22, 2) +
         (1.0 / 4.0) * pow(-helper14 * helper26 + helper20 * helper22, 2) +
         (1.0 / 4.0) * pow(helper16 * helper26 - helper20 * helper24, 2) +
         (1.0 / 4.0) * pow(helper17 * helper26 - helper18 * helper22, 2) +
         (1.0 / 4.0) * pow(helper27 * helper28 - helper29 * helper30, 2) +
         (1.0 / 4.0) * pow(-helper27 * helper32 + helper30 * helper31, 2) +
         (1.0 / 4.0) * pow(-helper28 * helper31 + helper29 * helper32, 2)) / denom
        ;
    return result_0;
}

void wmtk::harmonic_tet_jacobian(const std::array<double, 12>& T, Eigen::Vector3d& result)
{
    double helper_0 = T[10] * T[8];
    double helper_1 = T[11] * T[4];
    double helper_2 = T[5] * T[7];
    double helper_3 = T[10] * T[3];
    double helper_4 = T[10] * T[5];
    double helper_5 = T[11] * T[6];
    double helper_6 = T[11] * T[7];
    double helper_7 = T[3] * T[8];
    double helper_8 = T[5] * T[9];
    double helper_9 = T[4] * T[6];
    double helper_10 = T[7] * T[9];
    double helper_11 = T[4] * T[8];
    double helper_12 = T[10] * T[6];
    double helper_13 = T[11] * T[3];
    double helper_14 = T[5] * T[6];
    double helper_15 = T[8] * T[9];
    double helper_16 = T[3] * T[7];
    double helper_17 = T[4] * T[9];
    double helper_18 = T[0] * helper_0 + T[0] * helper_1 - T[0] * helper_11 + T[0] * helper_2 -
                       T[0] * helper_4 - T[0] * helper_6 - T[1] * helper_13 - T[1] * helper_14 -
                       T[1] * helper_15 + T[1] * helper_5 + T[1] * helper_7 + T[1] * helper_8 +
                       T[2] * helper_10 - T[2] * helper_12 - T[2] * helper_16 - T[2] * helper_17 +
                       T[2] * helper_3 + T[2] * helper_9 - T[3] * helper_0 + T[3] * helper_6 -
                       T[6] * helper_1 + T[6] * helper_4 + T[9] * helper_11 - T[9] * helper_2;
    double helper_19 = 1.0 / helper_18;
    double helper_20 = 2 * T[10];
    double helper_21 = -helper_20;
    double helper_22 = 2 * T[4];
    double helper_23 = T[0] - T[3];
    double helper_24 = -T[10];
    double helper_25 = T[1] + helper_24;
    double helper_26 = -T[9];
    double helper_27 = T[0] + helper_26;
    double helper_28 = T[1] - T[4];
    double helper_29 = helper_23 * helper_25 - helper_27 * helper_28;
    double helper_30 = (1.0 / 4.0) * helper_29;
    double helper_31 = 2 * T[7];
    double helper_32 = -T[6];
    double helper_33 = T[0] + helper_32;
    double helper_34 = -T[7];
    double helper_35 = T[1] + helper_34;
    double helper_36 = helper_25 * helper_33 - helper_27 * helper_35;
    double helper_37 = (1.0 / 4.0) * helper_36;
    double helper_38 = 2 * T[11];
    double helper_39 = 2 * T[5];
    double helper_40 = -helper_39;
    double helper_41 = T[2] - T[5];
    double helper_42 = -T[11];
    double helper_43 = T[2] + helper_42;
    double helper_44 = -helper_23 * helper_43 + helper_27 * helper_41;
    double helper_45 = (1.0 / 4.0) * helper_44;
    double helper_46 = 2 * T[8];
    double helper_47 = -helper_46;
    double helper_48 = -T[8];
    double helper_49 = T[2] + helper_48;
    double helper_50 = helper_27 * helper_49 - helper_33 * helper_43;
    double helper_51 = (1.0 / 4.0) * helper_50;
    double helper_52 = -helper_31;
    double helper_53 = helper_23 * helper_35 - helper_28 * helper_33;
    double helper_54 = (1.0 / 4.0) * helper_53;
    double helper_55 = -helper_23 * helper_49 + helper_33 * helper_41;
    double helper_56 = (1.0 / 4.0) * helper_55;
    double helper_57 = -helper_25 * helper_41 + helper_28 * helper_43;
    double helper_58 = -helper_25 * helper_49 + helper_35 * helper_43;
    double helper_59 = T[4] + helper_24;
    double helper_60 = T[3] + helper_32;
    double helper_61 = T[3] + helper_26;
    double helper_62 = T[4] + helper_34;
    double helper_63 = T[5] + helper_42;
    double helper_64 = T[5] + helper_48;
    double helper_65 = helper_28 * helper_49 - helper_35 * helper_41;
    double helper_66 = ((1.0 / 4.0) * pow(helper_29, 2) + (1.0 / 4.0) * pow(helper_36, 2) +
                        (1.0 / 4.0) * pow(helper_44, 2) + (1.0 / 4.0) * pow(helper_50, 2) +
                        (1.0 / 4.0) * pow(helper_53, 2) + (1.0 / 4.0) * pow(helper_55, 2) +
                        (1.0 / 4.0) * pow(helper_57, 2) + (1.0 / 4.0) * pow(helper_58, 2) +
                        (1.0 / 4.0) * pow(helper_65, 2) +
                        (1.0 / 4.0) * pow(helper_59 * helper_60 - helper_61 * helper_62, 2) +
                        (1.0 / 4.0) * pow(-helper_59 * helper_64 + helper_62 * helper_63, 2) +
                        (1.0 / 4.0) * pow(-helper_60 * helper_63 + helper_61 * helper_64, 2)) /
                       pow(helper_18, 2);
    double helper_67 = -helper_38;
    double helper_68 = (1.0 / 4.0) * helper_57;
    double helper_69 = (1.0 / 4.0) * helper_58;
    double helper_70 = 2 * T[3];
    double helper_71 = -helper_70;
    double helper_72 = 2 * T[6];
    double helper_73 = 2 * T[9];
    double helper_74 = (1.0 / 4.0) * helper_65;
    double helper_75 = -helper_72;
    double helper_76 = -helper_22;
    double helper_77 = -helper_73;
    auto result_0 =
        helper_19 * (helper_30 * (helper_21 + helper_22) + helper_37 * (helper_21 + helper_31) +
                     helper_45 * (helper_38 + helper_40) + helper_51 * (helper_38 + helper_47) +
                     helper_54 * (helper_22 + helper_52) + helper_56 * (helper_40 + helper_46)) +
        helper_66 * (-helper_0 - helper_1 + helper_11 - helper_2 + helper_4 + helper_6);
    auto result_1 =
        helper_19 * (helper_30 * (helper_71 + helper_73) + helper_37 * (helper_73 + helper_75) +
                     helper_54 * (helper_71 + helper_72) + helper_68 * (helper_39 + helper_67) +
                     helper_69 * (helper_46 + helper_67) + helper_74 * (helper_39 + helper_47)) +
        helper_66 * (helper_13 + helper_14 + helper_15 - helper_5 - helper_7 - helper_8);
    auto result_2 =
        helper_19 * (helper_45 * (helper_70 + helper_77) + helper_51 * (helper_72 + helper_77) +
                     helper_56 * (helper_70 + helper_75) + helper_68 * (helper_20 + helper_76) +
                     helper_69 * (helper_20 + helper_52) + helper_74 * (helper_31 + helper_76)) +
        helper_66 * (-helper_10 + helper_12 + helper_16 + helper_17 - helper_3 - helper_9);

    result = Eigen::Vector3d(result_0, result_1, result_2);
}