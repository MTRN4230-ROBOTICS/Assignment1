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
% Focal Length:          fc = [ 1314.03448   1314.03448 ]
% Principal point:       cc = [ 799.50000   599.50000 ]
% Skew:             alpha_c = [ 0.00000 ]   => angle of pixel = 90.00000 degrees
% Distortion:            kc = [ 0.00000   0.00000   0.00000   0.00000   0.00000 ]
% 
% Main calibration optimization procedure - Number of images: 20
% Gradient descent iterations: 1...2...3...4...5...6...7...8...9...10...11...12...13...14...15...16...17...18...19...20...done
% Estimation of uncertainties...done
% 
% 
% Calibration results after optimization (with uncertainties):
% 
% Focal Length:          fc = [ 1324.72530   1332.78177 ] ± [ 4.92135   5.25005 ]
% Principal point:       cc = [ 770.06216   578.85253 ] ± [ 13.18775   10.18814 ]
% Skew:             alpha_c = [ 0.00000 ] ± [ 0.00000  ]   => angle of pixel axes = 90.00000 ± 0.00000 degrees
% Distortion:            kc = [ 0.05932   -0.06121   -0.00426   0.00061  0.00000 ] ± [ 0.04394   0.47888   0.00316   0.00408  0.00000 ]
% Pixel error:          err = [ 0.32413   0.51564 ]
% 
% Note: The numerical errors are approximately three times the standard deviations (for reference).
% 
% 
% Recommendation: Some distortion coefficients are found equal to zero (within their uncertainties).
%                 To reject them from the optimization set est_dist=[1;0;1;1;0] and run Calibration