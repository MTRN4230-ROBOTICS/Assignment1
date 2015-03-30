% Aspect ratio optimized (est_aspect_ratio = 1) -> both components of fc are estimated (DEFAULT).
% Principal point optimized (center_optim=1) - (DEFAULT). To reject principal point, set center_optim=0
% Skew not optimized (est_alpha=0) - (DEFAULT)
% Distortion not fully estimated (defined by the variable est_dist):
%      Sixth order distortion not estimated (est_dist(5)=0) - (DEFAULT) .
% Initialization of the principal point at the center of the image.
% Initialization of the intrinsic parameters using the vanishing points of planar patterns.
% 
% Initialization of the intrinsic parameters - Number of images: 20
% 
% 
% Calibration parameters after initialization:
% 
% Focal Length:          fc = [ 1344.10479   1344.10479 ]
% Principal point:       cc = [ 799.50000   599.50000 ]
% Skew:             alpha_c = [ 0.00000 ]   => angle of pixel = 90.00000 degrees
% Distortion:            kc = [ 0.00000   0.00000   0.00000   0.00000   0.00000 ]
% 
% Main calibration optimization procedure - Number of images: 20
% Gradient descent iterations: 1...2...3...4...5...6...7...8...9...10...11...12...13...14...15...16...17...18...19...done
% Estimation of uncertainties...done
% 
% 
% Calibration results after optimization (with uncertainties):
% 
% Focal Length:          fc = [ 1330.39802   1337.96219 ] ± [ 5.01223   5.00935 ]
% Principal point:       cc = [ 781.12515   561.73763 ] ± [ 8.62489   7.00738 ]
% Skew:             alpha_c = [ 0.00000 ] ± [ 0.00000  ]   => angle of pixel axes = 90.00000 ± 0.00000 degrees
% Distortion:            kc = [ 0.07245   -0.19544   -0.00120   0.00133  0.00000 ] ± [ 0.01782   0.10237   0.00212   0.00257  0.00000 ]
% Pixel error:          err = [ 0.34207   0.28464 ]
% 
% Note: The numerical errors are approximately three times the standard deviations (for reference).