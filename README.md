# Autonomous Mobile Robots
## Course Structure :space_invader:
    
- Section 0 :alien:	
    - Introduction [:books:](lectures/amr_introduction.pdf) 
- Section 1 (Control) :alien:
	- Motion Control  [:books:](lectures/amr_motion.pdf) 
		<ul>
			<li>Kinematics of wheeled mobile robots: internal, external, direct, and inverse </li>
				<ul>
				<li>Differential drive kinematics</li>
				<li>Bicycle drive kinematics</li>
				<li>Rear-wheel bicycle drive kinematics</li>
				<li>Car(Ackermann) drive kinematics</li>
				</ul>
			<li>Wheel kinematics constraints: rolling contact  and lateral slippage </li>
			<li>Wheeled Mobile System Control: pose and orientation
				<ul>
				<li>Control to reference pose</li>
				<li>Control to reference pose via an intermediate point</li>
				<li>Control to reference pose via an intermediate direction</li>
				<li>Control by a straight line and a circular arc</li>
				<li>Reference path control</li>
				</ul>
			</li>
		</ul>
	- Dubins path planning [:books:](lectures/amr_dubins_path_planning.pdf) 
- Section 2 (Estimation) :alien:	
    - Bayesian Filter [:books: ](lectures/amr_bayesian_filter.pdf)  
	 	<ul>
	      <li>Basic of Probability</li>
	      <li>Probabilistic Generative Laws</li>
	      <li>Estimation from Measurements</li>
	      <li>Estimation from Measurements and Controls</li>
	    </ul>

	- Kalman filter [:books:](lectures/amr_kalman_filter.pdf)
		<ul>
			<li> Gaussian Distribution</li>
			<li> One Dimensional Kalman Filter</li>
			<li> Multivariate Density Function</li>
			<li> Marginal Density Function</li>
			<li> Multivariate Normal Function</li>
			<li> Two Dimensional Gaussian</li>
			<li> Multiple Random Variable</li>
			<li> Multidimensional Kalman Filter</li>
			<li> Sensor Fusion</li>
			<li> Linearization, Taylor Series Expansion, Linear Systems</li>
			<li> Extended Kalman Filter (EKF)</li>
			<li> Comparison between KF and EKF</li>
		</ul>

	- Particle Filter [:books: ](lectures/amr_particle_filter.pdf) 
		<ul>
			<li> A Taxonomy of Particle Filter </li>
			<li> Bayesian Filter </li>
			<li> Monte Carlo Integration (MCI) </li>
			<li> Particle Filter </li>
			<li> Importance Sampling </li>
			<li> Particle Filter Algorithm </li>
		</ul>

	- Robot localization   [:books: ](lectures/amr_robot_localization.pdf) 
		<ul>
			<li> A Taxonomy of Localization Problems </li>
			<li> Markov localization  </li>
			<li> Environment Sensing </li>
			<li> Motion in the Environment </li>
			<li> Localization in the Environment </li>
			<li> EKF localization with known correspondence </li>
			<li> Particle filter localization with known correspondence </li>
		</ul>
	
	- Robot mapping   [:books: ](lectures/amr_mapping.pdf) 
		<ul>
			<li> Ray casting and ray tracing </li>
			<li> Ray-casting algorithm  </li>
			<li> Winding number algorithm </li>
			<li> TODO (more to come) </li>
		</ul>

	- Robot simultaneous localization and
mapping (SLAM)   [:books: ](lectures/amr_mapping_and_localization.pdf) 
		<ul>
			<li> Introduction </li>
			<li> TODO (more to come)  </li>
		</ul>

- Section 3 (Perception) :alien:
	- Line Extraction Techniques [:books:](lectures/amr_feature_extraction.pdf)
				<ul>
					<li>Hough Transformation</li>
					<li>Split-and-Merge Algorithm</li>
					<li>Line Regression Algorithm/li>
				</ul>
	- Similarity Measurements [:books:](lectures/amr_feature_extraction.pdf)
		<ul>
			<li>Edge Detection (based on derivative and gradient)</li>
			<li> Corner Detection</li>
			<li> The Laplace Operator </li>
			<li>  Laplacian of Gaussian (LoG) </li>
			<li>  Difference of Gaussian (DoG) </li>
			<li>  Gaussian and Laplacian Pyramids </li>
			<li>  Scale Invariant Feature Transform (SIFT) </li> 
				<ul>
					<li>Scale-space Extrema Detection</li>
					<li>Keypoint Localization</li>
					<li>Orientation Assignment</li>
					<li>Keypoint Descriptor</li>
				</ul>
- Monocular Vision [:books:](lectures/amr_vision.pdf)
	<ul>
		<li>Pinhole Camera Model</li>
		<li>Image Plane, Camera Plane, Projection Matrix</li>
		<li>Projective transformation</li>
		<li>Finding Projection Matrix using Direct Linear Transform (DLT)</li>
		<li>Camera Calibration</li>
	</ul>
- Stereo Vision [:books:](lectures/amr_vision.pdf)
	<ul>
		<li>Simple Stereo, General Stereo</li>
		<li>Some homogeneous properties</li>
		<li>Epipolar Geometry</li>
		<li>Essential matrix, Fundamental matrix</li>
		<li>Camera Calibration</li>
	</ul>
		<li> Depth Estimation </li>
	</ul>
- References [:books:]	
	<ul>
			<li> Robert Grover Brown, Patrick YC Hwang, et al.
		Introduction to random signals and applied Kalman filtering,
		volume 3.
		Wiley New York, 1992.</li>
			<li> Gregor Klancar, Andrej Zdesar, Saso Blazic, and Igor Skrjanc.
		Wheeled mobile robotics: from fundamentals towards
		autonomous systems.
		Butterworth-Heinemann, 2017. </li>
			<li> Roland Siegwart, Illah Reza Nourbakhsh, and Davide Scaramuzza.
		Introduction to autonomous mobile robots.
		MIT press, 2011. </li>
			<li> Sebastian Thrun.
		Probabilistic robotics.
		Communications of the ACM, 45(3):52–57, 2002. </li>
			<li> https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python </li>
	</ul> 




