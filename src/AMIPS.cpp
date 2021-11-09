//
// Created by Yixin Hu on 11/9/21.
//

#include "AMIPS.h"

wmtk::Scalar wmtk::AMIPS_energy(const std::array<Scalar, 12>& T) {
    Scalar helper_0[12];
    helper_0[0] = T[0];
    helper_0[1] = T[1];
    helper_0[2] = T[2];
    helper_0[3] = T[3];
    helper_0[4] = T[4];
    helper_0[5] = T[5];
    helper_0[6] = T[6];
    helper_0[7] = T[7];
    helper_0[8] = T[8];
    helper_0[9] = T[9];
    helper_0[10] = T[10];
    helper_0[11] = T[11];
    Scalar helper_1 = helper_0[2];
    Scalar helper_2 = helper_0[11];
    Scalar helper_3 = helper_0[0];
    Scalar helper_4 = helper_0[3];
    Scalar helper_5 = helper_0[9];
    Scalar helper_6 = 0.577350269189626 * helper_3 - 1.15470053837925 * helper_4 + 0.577350269189626 * helper_5;
    Scalar helper_7 = helper_0[1];
    Scalar helper_8 = helper_0[4];
    Scalar helper_9 = helper_0[7];
    Scalar helper_10 = helper_0[10];
    Scalar helper_11 = 0.408248290463863 * helper_10 + 0.408248290463863 * helper_7 + 0.408248290463863 * helper_8 -
                       1.22474487139159 * helper_9;
    Scalar helper_12 = 0.577350269189626 * helper_10 + 0.577350269189626 * helper_7 - 1.15470053837925 * helper_8;
    Scalar helper_13 = helper_0[6];
    Scalar helper_14 = -1.22474487139159 * helper_13 + 0.408248290463863 * helper_3 + 0.408248290463863 * helper_4 +
                       0.408248290463863 * helper_5;
    Scalar helper_15 = helper_0[5];
    Scalar helper_16 = helper_0[8];
    Scalar helper_17 = 0.408248290463863 * helper_1 + 0.408248290463863 * helper_15 - 1.22474487139159 * helper_16 +
                       0.408248290463863 * helper_2;
    Scalar helper_18 = 0.577350269189626 * helper_1 - 1.15470053837925 * helper_15 + 0.577350269189626 * helper_2;
    Scalar helper_19 = 0.5 * helper_13 + 0.5 * helper_4;
    Scalar helper_20 = 0.5 * helper_8 + 0.5 * helper_9;
    Scalar helper_21 = 0.5 * helper_15 + 0.5 * helper_16;
    Scalar helper_22 = (helper_1 - helper_2) * (helper_11 * helper_6 - helper_12 * helper_14) -
                       (-helper_10 + helper_7) * (-helper_14 * helper_18 + helper_17 * helper_6) +
                       (helper_3 - helper_5) * (-helper_11 * helper_18 + helper_12 * helper_17);
    Scalar res = -(helper_1 * (-1.5 * helper_1 + 0.5 * helper_2 + helper_21) +
                   helper_10 * (-1.5 * helper_10 + helper_20 + 0.5 * helper_7) +
                   helper_13 * (-1.5 * helper_13 + 0.5 * helper_3 + 0.5 * helper_4 + 0.5 * helper_5) +
                   helper_15 * (0.5 * helper_1 - 1.5 * helper_15 + 0.5 * helper_16 + 0.5 * helper_2) +
                   helper_16 * (0.5 * helper_1 + 0.5 * helper_15 - 1.5 * helper_16 + 0.5 * helper_2) +
                   helper_2 * (0.5 * helper_1 - 1.5 * helper_2 + helper_21) +
                   helper_3 * (helper_19 - 1.5 * helper_3 + 0.5 * helper_5) +
                   helper_4 * (0.5 * helper_13 + 0.5 * helper_3 - 1.5 * helper_4 + 0.5 * helper_5) +
                   helper_5 * (helper_19 + 0.5 * helper_3 - 1.5 * helper_5) +
                   helper_7 * (0.5 * helper_10 + helper_20 - 1.5 * helper_7) +
                   helper_8 * (0.5 * helper_10 + 0.5 * helper_7 - 1.5 * helper_8 + 0.5 * helper_9) +
                   helper_9 * (0.5 * helper_10 + 0.5 * helper_7 + 0.5 * helper_8 - 1.5 * helper_9))
                 / std::cbrt(helper_22*helper_22);
//                 * pow(pow((helper_1 - helper_2) * (helper_11 * helper_6 - helper_12 * helper_14) -
//                         (-helper_10 + helper_7) * (-helper_14 * helper_18 + helper_17 * helper_6) +
//                         (helper_3 - helper_5) * (-helper_11 * helper_18 + helper_12 * helper_17), 2),
//                     -0.333333333333333);
    return res;
}

void wmtk::AMIPS_jacobian(const std::array<Scalar, 12>& T, Vector3& result_0){
    Scalar helper_0[12];
    helper_0[0] = T[0];
    helper_0[1] = T[1];
    helper_0[2] = T[2];
    helper_0[3] = T[3];
    helper_0[4] = T[4];
    helper_0[5] = T[5];
    helper_0[6] = T[6];
    helper_0[7] = T[7];
    helper_0[8] = T[8];
    helper_0[9] = T[9];
    helper_0[10] = T[10];
    helper_0[11] = T[11];
    Scalar helper_1 = helper_0[1];
    Scalar helper_2 = helper_0[10];
    Scalar helper_3 = helper_1 - helper_2;
    Scalar helper_4 = helper_0[0];
    Scalar helper_5 = helper_0[3];
    Scalar helper_6 = helper_0[9];
    Scalar helper_7 = 0.577350269189626*helper_4 - 1.15470053837925*helper_5 + 0.577350269189626*helper_6;
    Scalar helper_8 = helper_0[2];
    Scalar helper_9 = 0.408248290463863*helper_8;
    Scalar helper_10 = helper_0[5];
    Scalar helper_11 = 0.408248290463863*helper_10;
    Scalar helper_12 = helper_0[8];
    Scalar helper_13 = 1.22474487139159*helper_12;
    Scalar helper_14 = helper_0[11];
    Scalar helper_15 = 0.408248290463863*helper_14;
    Scalar helper_16 = helper_11 - helper_13 + helper_15 + helper_9;
    Scalar helper_17 = 0.577350269189626*helper_8;
    Scalar helper_18 = 1.15470053837925*helper_10;
    Scalar helper_19 = 0.577350269189626*helper_14;
    Scalar helper_20 = helper_17 - helper_18 + helper_19;
    Scalar helper_21 = helper_0[6];
    Scalar helper_22 = -1.22474487139159*helper_21 + 0.408248290463863*helper_4 + 0.408248290463863*helper_5 + 0.408248290463863*helper_6;
    Scalar helper_23 = helper_16*helper_7 - helper_20*helper_22;
    Scalar helper_24 = -helper_14 + helper_8;
    Scalar helper_25 = 0.408248290463863*helper_1;
    Scalar helper_26 = helper_0[4];
    Scalar helper_27 = 0.408248290463863*helper_26;
    Scalar helper_28 = helper_0[7];
    Scalar helper_29 = 1.22474487139159*helper_28;
    Scalar helper_30 = 0.408248290463863*helper_2;
    Scalar helper_31 = helper_25 + helper_27 - helper_29 + helper_30;
    Scalar helper_32 = helper_31*helper_7;
    Scalar helper_33 = 0.577350269189626*helper_1;
    Scalar helper_34 = 1.15470053837925*helper_26;
    Scalar helper_35 = 0.577350269189626*helper_2;
    Scalar helper_36 = helper_33 - helper_34 + helper_35;
    Scalar helper_37 = helper_22*helper_36;
    Scalar helper_38 = helper_4 - helper_6;
    Scalar helper_39 = helper_23*helper_3 - helper_24*(helper_32 - helper_37) - helper_38*(helper_16*helper_36 - helper_20*helper_31);
    Scalar helper_40 = pow(pow(helper_39, 2), -0.333333333333333);
    Scalar helper_41 = 0.707106781186548*helper_10 - 0.707106781186548*helper_12;
    Scalar helper_42 = 0.707106781186548*helper_26 - 0.707106781186548*helper_28;
    Scalar helper_43 = 0.5*helper_21 + 0.5*helper_5;
    Scalar helper_44 = 0.5*helper_26 + 0.5*helper_28;
    Scalar helper_45 = 0.5*helper_10 + 0.5*helper_12;
    Scalar helper_46 = 0.666666666666667*(helper_1*(-1.5*helper_1 + 0.5*helper_2 + helper_44) + helper_10*(-1.5*helper_10 + 0.5*helper_12 + 0.5*helper_14 + 0.5*helper_8) + helper_12*(0.5*helper_10 - 1.5*helper_12 + 0.5*helper_14 + 0.5*helper_8) + helper_14*(-1.5*helper_14 + helper_45 + 0.5*helper_8) + helper_2*(0.5*helper_1 - 1.5*helper_2 + helper_44) + helper_21*(-1.5*helper_21 + 0.5*helper_4 + 0.5*helper_5 + 0.5*helper_6) + helper_26*(0.5*helper_1 + 0.5*helper_2 - 1.5*helper_26 + 0.5*helper_28) + helper_28*(0.5*helper_1 + 0.5*helper_2 + 0.5*helper_26 - 1.5*helper_28) + helper_4*(-1.5*helper_4 + helper_43 + 0.5*helper_6) + helper_5*(0.5*helper_21 + 0.5*helper_4 - 1.5*helper_5 + 0.5*helper_6) + helper_6*(0.5*helper_4 + helper_43 - 1.5*helper_6) + helper_8*(0.5*helper_14 + helper_45 - 1.5*helper_8))/helper_39;
    Scalar helper_47 = -0.707106781186548*helper_21 + 0.707106781186548*helper_5;
    result_0[0] = -helper_40*(1.0*helper_21 - 3.0*helper_4 + helper_46*(helper_41*(-helper_1 + helper_2) - helper_42*(helper_14 - helper_8) - (-helper_17 + helper_18 - helper_19)*(-helper_25 - helper_27 + helper_29 - helper_30) + (-helper_33 + helper_34 - helper_35)*(-helper_11 + helper_13 - helper_15 - helper_9)) + 1.0*helper_5 + 1.0*helper_6);
    result_0[1] = helper_40*(3.0*helper_1 - 1.0*helper_2 - 1.0*helper_26 - 1.0*helper_28 + helper_46*(helper_23 + helper_24*helper_47 - helper_38*helper_41));
    result_0[2] = helper_40*(-1.0*helper_10 - 1.0*helper_12 - 1.0*helper_14 + helper_46*(-helper_3*helper_47 - helper_32 + helper_37 + helper_38*helper_42) + 3.0*helper_8);
}

void wmtk::AMIPS_hessian(const std::array<Scalar, 12>& T, Matrix3& result_0){
    Scalar helper_0[12];
    helper_0[0] = T[0];
    helper_0[1] = T[1];
    helper_0[2] = T[2];
    helper_0[3] = T[3];
    helper_0[4] = T[4];
    helper_0[5] = T[5];
    helper_0[6] = T[6];
    helper_0[7] = T[7];
    helper_0[8] = T[8];
    helper_0[9] = T[9];
    helper_0[10] = T[10];
    helper_0[11] = T[11];
    Scalar helper_1 = helper_0[2];
    Scalar helper_2 = helper_0[11];
    Scalar helper_3 = helper_1 - helper_2;
    Scalar helper_4 = helper_0[0];
    Scalar helper_5 = 0.577350269189626*helper_4;
    Scalar helper_6 = helper_0[3];
    Scalar helper_7 = 1.15470053837925*helper_6;
    Scalar helper_8 = helper_0[9];
    Scalar helper_9 = 0.577350269189626*helper_8;
    Scalar helper_10 = helper_5 - helper_7 + helper_9;
    Scalar helper_11 = helper_0[1];
    Scalar helper_12 = 0.408248290463863*helper_11;
    Scalar helper_13 = helper_0[4];
    Scalar helper_14 = 0.408248290463863*helper_13;
    Scalar helper_15 = helper_0[7];
    Scalar helper_16 = 1.22474487139159*helper_15;
    Scalar helper_17 = helper_0[10];
    Scalar helper_18 = 0.408248290463863*helper_17;
    Scalar helper_19 = helper_12 + helper_14 - helper_16 + helper_18;
    Scalar helper_20 = helper_10*helper_19;
    Scalar helper_21 = 0.577350269189626*helper_11;
    Scalar helper_22 = 1.15470053837925*helper_13;
    Scalar helper_23 = 0.577350269189626*helper_17;
    Scalar helper_24 = helper_21 - helper_22 + helper_23;
    Scalar helper_25 = 0.408248290463863*helper_4;
    Scalar helper_26 = 0.408248290463863*helper_6;
    Scalar helper_27 = helper_0[6];
    Scalar helper_28 = 1.22474487139159*helper_27;
    Scalar helper_29 = 0.408248290463863*helper_8;
    Scalar helper_30 = helper_25 + helper_26 - helper_28 + helper_29;
    Scalar helper_31 = helper_24*helper_30;
    Scalar helper_32 = helper_3*(helper_20 - helper_31);
    Scalar helper_33 = helper_4 - helper_8;
    Scalar helper_34 = 0.408248290463863*helper_1;
    Scalar helper_35 = helper_0[5];
    Scalar helper_36 = 0.408248290463863*helper_35;
    Scalar helper_37 = helper_0[8];
    Scalar helper_38 = 1.22474487139159*helper_37;
    Scalar helper_39 = 0.408248290463863*helper_2;
    Scalar helper_40 = helper_34 + helper_36 - helper_38 + helper_39;
    Scalar helper_41 = helper_24*helper_40;
    Scalar helper_42 = 0.577350269189626*helper_1;
    Scalar helper_43 = 1.15470053837925*helper_35;
    Scalar helper_44 = 0.577350269189626*helper_2;
    Scalar helper_45 = helper_42 - helper_43 + helper_44;
    Scalar helper_46 = helper_19*helper_45;
    Scalar helper_47 = helper_41 - helper_46;
    Scalar helper_48 = helper_33*helper_47;
    Scalar helper_49 = helper_11 - helper_17;
    Scalar helper_50 = helper_10*helper_40;
    Scalar helper_51 = helper_30*helper_45;
    Scalar helper_52 = helper_50 - helper_51;
    Scalar helper_53 = helper_49*helper_52;
    Scalar helper_54 = helper_32 + helper_48 - helper_53;
    Scalar helper_55 = pow(helper_54, 2);
    Scalar helper_56 = pow(helper_55, -0.333333333333333);
    Scalar helper_57 = 1.0*helper_27 - 3.0*helper_4 + 1.0*helper_6 + 1.0*helper_8;
    Scalar helper_58 = 0.707106781186548*helper_13;
    Scalar helper_59 = 0.707106781186548*helper_15;
    Scalar helper_60 = helper_58 - helper_59;
    Scalar helper_61 = helper_3*helper_60;
    Scalar helper_62 = 0.707106781186548*helper_35 - 0.707106781186548*helper_37;
    Scalar helper_63 = helper_49*helper_62;
    Scalar helper_64 = helper_47 + helper_61 - helper_63;
    Scalar helper_65 = 1.33333333333333/helper_54;
    Scalar helper_66 = 1.0/helper_55;
    Scalar helper_67 = 0.5*helper_27 + 0.5*helper_6;
    Scalar helper_68 = -1.5*helper_4 + helper_67 + 0.5*helper_8;
    Scalar helper_69 = 0.5*helper_4 + helper_67 - 1.5*helper_8;
    Scalar helper_70 = -1.5*helper_27 + 0.5*helper_4 + 0.5*helper_6 + 0.5*helper_8;
    Scalar helper_71 = 0.5*helper_27 + 0.5*helper_4 - 1.5*helper_6 + 0.5*helper_8;
    Scalar helper_72 = 0.5*helper_13 + 0.5*helper_15;
    Scalar helper_73 = -1.5*helper_11 + 0.5*helper_17 + helper_72;
    Scalar helper_74 = 0.5*helper_11 - 1.5*helper_17 + helper_72;
    Scalar helper_75 = 0.5*helper_11 + 0.5*helper_13 - 1.5*helper_15 + 0.5*helper_17;
    Scalar helper_76 = 0.5*helper_11 - 1.5*helper_13 + 0.5*helper_15 + 0.5*helper_17;
    Scalar helper_77 = 0.5*helper_35 + 0.5*helper_37;
    Scalar helper_78 = -1.5*helper_1 + 0.5*helper_2 + helper_77;
    Scalar helper_79 = 0.5*helper_1 - 1.5*helper_2 + helper_77;
    Scalar helper_80 = 0.5*helper_1 + 0.5*helper_2 + 0.5*helper_35 - 1.5*helper_37;
    Scalar helper_81 = 0.5*helper_1 + 0.5*helper_2 - 1.5*helper_35 + 0.5*helper_37;
    Scalar helper_82 = helper_1*helper_78 + helper_11*helper_73 + helper_13*helper_76 + helper_15*helper_75 + helper_17*helper_74 + helper_2*helper_79 + helper_27*helper_70 + helper_35*helper_81 + helper_37*helper_80 + helper_4*helper_68 + helper_6*helper_71 + helper_69*helper_8;
    Scalar helper_83 = 0.444444444444444*helper_66*helper_82;
    Scalar helper_84 = helper_66*helper_82;
    Scalar helper_85 = -helper_32 - helper_48 + helper_53;
    Scalar helper_86 = 1.0/helper_85;
    Scalar helper_87 = helper_86*pow(pow(helper_85, 2), -0.333333333333333);
    Scalar helper_88 = 0.707106781186548*helper_6;
    Scalar helper_89 = 0.707106781186548*helper_27;
    Scalar helper_90 = helper_88 - helper_89;
    Scalar helper_91 = 0.666666666666667*helper_10*helper_40 + 0.666666666666667*helper_3*helper_90 - 0.666666666666667*helper_30*helper_45 - 0.666666666666667*helper_33*helper_62;
    Scalar helper_92 = -3.0*helper_11 + 1.0*helper_13 + 1.0*helper_15 + 1.0*helper_17;
    Scalar helper_93 = -helper_11 + helper_17;
    Scalar helper_94 = -helper_1 + helper_2;
    Scalar helper_95 = -helper_21 + helper_22 - helper_23;
    Scalar helper_96 = -helper_34 - helper_36 + helper_38 - helper_39;
    Scalar helper_97 = -helper_42 + helper_43 - helper_44;
    Scalar helper_98 = -helper_12 - helper_14 + helper_16 - helper_18;
    Scalar helper_99 = -0.666666666666667*helper_60*helper_94 + 0.666666666666667*helper_62*helper_93 + 0.666666666666667*helper_95*helper_96 - 0.666666666666667*helper_97*helper_98;
    Scalar helper_100 = helper_3*helper_90;
    Scalar helper_101 = helper_33*helper_62;
    Scalar helper_102 = helper_100 - helper_101 + helper_52;
    Scalar helper_103 = -helper_60*helper_94 + helper_62*helper_93 + helper_95*helper_96 - helper_97*helper_98;
    Scalar helper_104 = 0.444444444444444*helper_102*helper_103*helper_82*helper_86 + helper_57*helper_91 - helper_92*helper_99;
    Scalar helper_105 = 1.85037170770859e-17*helper_1*helper_78 + 1.85037170770859e-17*helper_11*helper_73 + 1.85037170770859e-17*helper_13*helper_76 + 1.85037170770859e-17*helper_15*helper_75 + 1.85037170770859e-17*helper_17*helper_74 + 1.85037170770859e-17*helper_2*helper_79 + 1.85037170770859e-17*helper_27*helper_70 + 1.85037170770859e-17*helper_35*helper_81 + 1.85037170770859e-17*helper_37*helper_80 + 1.85037170770859e-17*helper_4*helper_68 + 1.85037170770859e-17*helper_6*helper_71 + 1.85037170770859e-17*helper_69*helper_8;
    Scalar helper_106 = helper_64*helper_82*helper_86;
    Scalar helper_107 = -0.666666666666667*helper_10*helper_19 + 0.666666666666667*helper_24*helper_30 + 0.666666666666667*helper_33*helper_60 - 0.666666666666667*helper_49*helper_90;
    Scalar helper_108 = -3.0*helper_1 + 1.0*helper_2 + 1.0*helper_35 + 1.0*helper_37;
    Scalar helper_109 = -helper_20 + helper_31 + helper_33*helper_60 - helper_49*helper_90;
    Scalar helper_110 = 0.444444444444444*helper_109*helper_82*helper_86;
    Scalar helper_111 = helper_103*helper_110 + helper_107*helper_57 - helper_108*helper_99;
    Scalar helper_112 = -helper_4 + helper_8;
    Scalar helper_113 = -helper_88 + helper_89;
    Scalar helper_114 = -helper_5 + helper_7 - helper_9;
    Scalar helper_115 = -helper_25 - helper_26 + helper_28 - helper_29;
    Scalar helper_116 = helper_82*helper_86*(helper_112*helper_62 + helper_113*helper_94 + helper_114*helper_96 - helper_115*helper_97);
    Scalar helper_117 = -helper_100 + helper_101 - helper_50 + helper_51;
    Scalar helper_118 = -helper_102*helper_110 + helper_107*helper_92 + helper_108*helper_91;
    Scalar helper_119 = helper_82*helper_86*(helper_112*(-helper_58 + helper_59) - helper_113*helper_93 - helper_114*helper_98 + helper_115*helper_95);
    result_0(0, 0) = helper_56*(helper_57*helper_64*helper_65 - pow(helper_64, 2)*helper_83 + 0.666666666666667*helper_64*helper_84*(-helper_41 + helper_46 - helper_61 + helper_63) + 3.0);
    result_0(0, 1) = helper_87*(helper_104 - helper_105*helper_35 + helper_106*helper_91);
    result_0(0, 2) = helper_87*(helper_106*helper_107 + helper_111);
    result_0(1, 0) = helper_87*(helper_104 + helper_116*helper_99);
    result_0(1, 1) = helper_56*(-pow(helper_117, 2)*helper_83 + helper_117*helper_65*helper_92 + helper_117*helper_84*helper_91 + 3.0);
    result_0(1, 2) = helper_87*(-helper_105*helper_6 - helper_107*helper_116 + helper_118);
    result_0(2, 0) = helper_87*(-helper_105*helper_13 + helper_111 + helper_119*helper_99);
    result_0(2, 1) = helper_87*(helper_118 - helper_119*helper_91);
    result_0(2, 2) = helper_56*(-helper_108*helper_109*helper_65 - 1.11111111111111*pow(helper_109, 2)*helper_84 + 3.0);
}
