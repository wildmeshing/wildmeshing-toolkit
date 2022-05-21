wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/39507.stl -o fig1-fat//out//10.tetra.msh -j 10 --max-its 25 --filter-with-input
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin fig1-fat//out//10.tetra.msh_final.msh fig1-fat//out//32.harmo.msh -j 32
wildmeshing-toolkit/build/app/remeshing_app input_data/39507.stl fig1-fat//out//32.remesh.obj -e 1e-3 -j 32 -r 0.01 -f 0
wildmeshing-toolkit/build/app/qslim_app input_data/39507.stl fig1-fat//out//32.qslim.obj -j 32 -t 5e-3
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//32.obj -j 32
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//16.obj -j 16
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//8.obj -j 8
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//4.obj -j 4
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//2.obj -j 2
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//1.obj -j 1
wildmeshing-toolkit/build/app/sec_app input_data/Sapphos_Head.stl fig4-statue-sec//out//0.obj -j 0
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//32.obj -j 32 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//16.obj -j 16 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//8.obj -j 8 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//4.obj -j 4 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//2.obj -j 2 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//1.obj -j 1 -t 0.01
wildmeshing-toolkit/build/app/qslim_app input_data/upsampled_golden.obj fig5-qslim-golden//out//0.obj -j 0 -t 0.01
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//0.obj -j 0 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//1.obj -j 1 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//2.obj -j 2 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//4.obj -j 4 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//8.obj -j 8 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//16.obj -j 16 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/120628.stl fig6-lucy-UniRem//out//32.obj -j 32 -r 0.01 -f 0
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//32.msh --harmonize -j 32
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//16.msh --harmonize -j 16
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//8.msh --harmonize -j 8
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//4.msh --harmonize -j 4
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//2.msh --harmonize -j 2
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//1.msh --harmonize -j 1
wildmeshing-toolkit/build/app/harmonic_tet/wmtk_harmonic_tet_bin input_data/gaussian_points_1M.ply fig7-gauss-harm//out//0.msh --harmonize -j 0
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//32.msh -j 32 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//16.msh -j 16 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//8.msh -j 8 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//4.msh -j 4 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//2.msh -j 2 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//1.msh -j 1 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/tetwild/tetwild -i input_data/dragon.off -o fig8-tw-sample//out//0.msh -j 0 --max-its 10 --sample-envelope
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//32.obj -e 1e-3 -j 32 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//16.obj -e 1e-3 -j 16 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//8.obj -e 1e-3 -j 8 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//4.obj -e 1e-3 -j 4 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//2.obj -e 1e-3 -j 2 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//1.obj -e 1e-3 -j 1 -t 1e-2
wildmeshing-toolkit/build/app/sec_app input_data/einstein_big.stl fig9-eins-secenv//out//0.obj -e 1e-3 -j 0 -t 1e-2
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//32.obj -e 1e-3 -j 32 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//16.obj -e 1e-3 -j 16 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//8.obj -e 1e-3 -j 8 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//4.obj -e 1e-3 -j 4 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//2.obj -e 1e-3 -j 2 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//1.obj -e 1e-3 -j 1 -r 0.01 -f 0
wildmeshing-toolkit/build/app/remeshing_app input_data/head_-_ported_-_scaled.stl fig10-headported-UniRemEnv//out//0.obj -e 1e-3 -j 0 -r 0.01 -f 0