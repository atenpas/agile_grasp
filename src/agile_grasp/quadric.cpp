#include <agile_grasp/quadric.h>

Quadric::Quadric(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& T_cams,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const Eigen::Vector3d& sample, bool is_deterministic) :
		input_(input), sample_(sample), is_deterministic_(is_deterministic)
{
	cam_origins_.resize(3, T_cams.size());
	for (int i = 0; i < cam_origins_.cols(); i++)
	{
		cam_origins_.col(i) = T_cams.at(i).block < 3, 1 > (0, 3);
	}
}

void Quadric::fitQuadric(const std::vector<int> &indices)
{
	int n = indices.size();

	// calculate matrices M and N
	Eigen::Matrix<double, TAUBIN_MATRICES_SIZE, TAUBIN_MATRICES_SIZE> M;
	Eigen::Matrix<double, TAUBIN_MATRICES_SIZE, TAUBIN_MATRICES_SIZE> N;
	M.setZero(10, 10);
	N.setZero(10, 10);

	for (int i = 0; i < n; i++)
	{
		if (isnan(input_->points[indices[i]].x))
			continue;

		double x = input_->points[indices[i]].x;
		double y = input_->points[indices[i]].y;
		double z = input_->points[indices[i]].z;
		double x2 = x * x;
		double y2 = y * y;
		double z2 = z * z;
		double xy = x * y;
		double yz = y * z;
		double xz = x * z;

		// required calculations for M
		M(0, 0) += x2 * x2;
		M(0, 1) += x2 * y2;
		M(0, 2) += x2 * z2;
		M(0, 3) += x2 * xy;
		M(0, 4) += x2 * yz;
		M(0, 5) += x2 * xz;
		M(0, 6) += x2 * x;
		M(0, 7) += x2 * y;
		M(0, 8) += x2 * z;
		M(0, 9) += x2;
		M(1, 1) += y2 * y2;
		M(1, 2) += y2 * z2;
		M(1, 3) += y2 * xy;
		M(1, 4) += y2 * yz;
		M(1, 5) += y2 * xz;
		M(1, 6) += y2 * x;
		M(1, 7) += y2 * y;
		M(1, 8) += y2 * z;
		M(1, 9) += y2;
		M(2, 2) += z2 * z2;
		M(2, 3) += z2 * xy;
		M(2, 4) += z2 * yz;
		M(2, 5) += z2 * xz;
		M(2, 6) += z2 * x;
		M(2, 7) += z2 * y;
		M(2, 8) += z2 * z;
		M(2, 9) += z2;
		M(3, 8) += x * yz;
		M(3, 9) += xy;
		M(4, 9) += yz;
		M(5, 9) += xz;
		M(6, 9) += x;
		M(7, 9) += y;
		M(8, 9) += z;

		// repeating elements in M
		M(3, 3) = M(0, 1);
		M(5, 5) = M(0, 2);
		M(3, 5) = M(0, 4);
		M(3, 6) = M(0, 7);
		M(5, 6) = M(0, 8);
		M(6, 6) = M(0, 9);

		M(4, 4) = M(1, 2);
		M(3, 4) = M(1, 5);
		M(3, 7) = M(1, 6);
		M(4, 7) = M(1, 8);
		M(7, 7) = M(1, 9);

		M(4, 5) = M(2, 3);
		M(5, 8) = M(2, 6);
		M(4, 8) = M(2, 7);
		M(8, 8) = M(2, 9);

		M(4, 6) = M(3, 8);
		M(5, 7) = M(3, 8);
		M(6, 7) = M(3, 9);

		M(7, 8) = M(4, 9);

		M(6, 8) = M(5, 9);

		// required calculations for N
		N(0, 0) += 4.0 * x2;
		N(0, 3) += 2.0 * xy;
		N(0, 5) += 2.0 * xz;
		N(0, 6) += 2.0 * x;

		N(1, 1) += 4.0 * y2;
		N(1, 3) += 2.0 * xy;
		N(1, 4) += 2.0 * yz;
		N(1, 7) += 2.0 * y;

		N(2, 2) += 4.0 * z2;
		N(2, 4) += 2.0 * yz;
		N(2, 5) += 2.0 * xz;
		N(2, 8) += 2.0 * z;

		N(3, 3) += x2 + y2;
		N(3, 4) += xz;
		N(3, 5) += yz;
		N(3, 6) += y;
		N(3, 7) += x;

		N(4, 4) += y2 + z2;
		N(4, 5) += xy;
		N(4, 7) += z;
		N(4, 8) += y;

		N(5, 5) += x2 + z2;
		N(5, 6) += z;
		N(5, 8) += x;
	}

	M(9, 9) = n;
	// reflect upper triangular part in lower triangular part
	M.triangularView<Eigen::StrictlyLower>() = M.triangularView<Eigen::StrictlyUpper>().transpose();
	N(6, 6) = n;
	N(7, 7) = n;
	N(8, 8) = n;
	// reflect upper triangular part in lower triangular part
	N.triangularView<Eigen::StrictlyLower>() = N.triangularView<Eigen::StrictlyUpper>().transpose();
	//~ std::cout<<"M:\n"<<M<<std::endl;
	//~ std::cout<<"N:\n"<<N<<std::endl;

	// solve generalized Eigen problem to find quadric parameters
	Eigen::MatrixXd eigen_vectors;
	Eigen::MatrixXd lambda;
	solveGeneralizedEigenProblem(M, N, eigen_vectors, lambda);
	Eigen::VectorXd eigen_values = lambda.col(0).cwiseQuotient(lambda.col(2));
	int min_index;
	eigen_values.segment(0, 9).minCoeff(&min_index);
	parameters_ = eigen_vectors.col(min_index);
	parameters_.segment(3, 3) *= 0.5;

	// compute centroid and covariance matrix of quadric
	unpackQuadric();
}

void Quadric::findTaubinNormalAxis(const std::vector<int> &indices, const Eigen::VectorXi& cam_source)
{
	// quadric parameters in implicit form
	double a = parameters_(0);
	double b = parameters_(1);
	double c = parameters_(2);
	double d = 2.0 * parameters_(3);
	double e = 2.0 * parameters_(4);
	double f = 2.0 * parameters_(5);
	double g = parameters_(6);
	double h = parameters_(7);
	double i = parameters_(8);

	// sample <num_samples> points near surface
	Eigen::Matrix3Xd samples;
	Eigen::VectorXi samples_cam_source;
	if (!is_deterministic_)
	{
		int num_samples = 50;
		samples.resize(3, num_samples);
		samples_cam_source.resize(num_samples);
		if (indices.size() > num_samples)
		{
			for (int t = 0; t < num_samples; t++)
			{
				int r = rand() % indices.size();
				while (isnan(input_->points[indices[r]].x))
				{
					r = rand() % indices.size();
				}

				samples.col(t) << input_->points[indices[r]].x, input_->points[indices[r]].y, input_->points[indices[r]].z;
				samples_cam_source(t) = cam_source(r);
			}
		}
		else
		{
			samples_cam_source = cam_source;
			samples.resize(3, indices.size());
			for (int t = 0; t < indices.size(); t++)
			{
				samples.col(t) << input_->points[indices[t]].x, input_->points[indices[t]].y, input_->points[indices[t]].z;
			}
		}
	}
	else
	{
		samples_cam_source = cam_source;
		samples.resize(3, indices.size());
		for (int t = 0; t < indices.size(); t++)
		{
			samples.col(t) << input_->points[indices[t]].x, input_->points[indices[t]].y, input_->points[indices[t]].z;
		}
	}

//	std::cout << "Took subset of points in neighborhood" << std::endl;

	// calculate camera source for majority of points
	Eigen::VectorXd num_source(this->cam_origins_.cols());
	num_source << 0, 0;
	for (int cami = 0; cami < samples_cam_source.size(); cami++)
	{
//    std::cout << "cami: " << cami << ", samples_cam_source(cami): " << samples_cam_source(cami) << std::endl;
		if (samples_cam_source(cami) == 0)
			num_source(0)++;else
		if (samples_cam_source(cami) == 1)
			num_source(1)++;}
num_source	.maxCoeff(&majority_cam_source_);
//  std::cout << "samples_cam_source.size(): " << samples_cam_source.size() << ", num_source: " << num_source.transpose() << ", majority_cam_source_: " << majority_cam_source_ << "\n";

	//~ std::cout<<"\n";
	//~ std::cout<<"samples_near_surf:\n"<<samples_near_surf<<std::endl;
	//~ std::cout<<"2.0*a*samples_near_surf.row(0):\n"<<2.0*a*samples_near_surf.row(0)<<"\n";
	//~ std::cout<<"2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2):\n"<<2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2)<<"\n";
	//~ std::cout<<"2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2).array():\n"<<(2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2)).array()<<"\n";
	//~ std::cout<<"2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2) + g:\n"<<(2.0*a*samples_near_surf.row(0) + d*samples_near_surf.row(1) + f*samples_near_surf.row(2)).array() + g<<"\n";
	//~ std::cout<<"g: "<<g<<std::endl;

	// calculate normals at each of these points
	Eigen::MatrixXd fx = (2.0 * a * samples.row(0) + d * samples.row(1) + f * samples.row(2)).array() + g;
	//~ std::cout<<"fx:\n"<<fx<<std::endl;
	Eigen::MatrixXd fy = (2.0 * b * samples.row(1) + d * samples.row(0) + e * samples.row(2)).array() + h;
	//~ std::cout<<"fy:\n"<<fy<<std::endl;
	Eigen::MatrixXd fz = (2.0 * c * samples.row(2) + e * samples.row(1) + f * samples.row(0)).array() + i;
	//~ std::cout<<"fz:\n"<<fz<<std::endl;
	Eigen::MatrixXd normals(3, samples.cols());
	normals << fx, fy, fz;
	Eigen::MatrixXd gradient_magnitude = ((normals.cwiseProduct(normals)).colwise().sum()).cwiseSqrt();
	normals = normals.cwiseQuotient(gradient_magnitude.replicate(3, 1));
//  std::cout << "normals:\n" << normals.cols() << std::endl;

	findAverageNormalAxis(normals);
}

void Quadric::print()
{
	std::cout << "sample: " << sample_.transpose() << std::endl;
	std::cout << "parameters: " << parameters_.transpose() << std::endl;
	std::cout << "normals_ratio: " << normals_ratio_ << std::endl;
	std::cout << "normals_axis: " << curvature_axis_.transpose() << std::endl;
	std::cout << "normals_average: " << normal_.transpose() << std::endl;
	std::cout << "binormal: " << binormal_.transpose() << std::endl;
}

void Quadric::findAverageNormalAxis(const Eigen::MatrixXd &normals)
{
	// calculate curvature axis
	Eigen::Matrix3d M = normals * normals.transpose();
	//~ std::cout<<"M:\n"<<M<<std::endl;
	Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(M);
	Eigen::Vector3d eigen_values = eigen_solver.eigenvalues().real();
	Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors().real();
	//~ std::cout<<"eigen_values:\n"<<eigen_values<<std::endl;
	//~ std::cout<<"eigen_vectors:\n"<<eigen_vectors<<std::endl;
	Eigen::Vector3d sorted_eigen_values = eigen_values;
	std::sort(sorted_eigen_values.data(), sorted_eigen_values.data() + sorted_eigen_values.size());
	//~ std::cout<<"eigen_values:\n"<<eigen_values<<std::endl;
	//~ std::cout<<"sorted eigen_values:\n"<<sorted_eigen_values<<std::endl;
	normals_ratio_ = sorted_eigen_values(1) / sorted_eigen_values(2);
	int min_index;
	eigen_values.minCoeff(&min_index);
	curvature_axis_ = eigen_vectors.col(min_index);

	// calculate surface normal
	int max_index;
	(normals.transpose() * normals).array().pow(6).colwise().sum().maxCoeff(&max_index);
	Eigen::Vector3d normpartial = (Eigen::MatrixXd::Identity(3, 3)
			- curvature_axis_ * curvature_axis_.transpose()) * normals.col(max_index);
	//~ std::cout<<"normpartial:\n"<<normpartial<<std::endl;
	normal_ = normpartial / normpartial.norm();

	// create binormal
	binormal_ = curvature_axis_.cross(normal_);

	// require normal and binormal to be oriented towards source
	Eigen::Vector3d source_to_sample = sample_ - cam_origins_.col(majority_cam_source_);
//  std::cout << source_to_sample.transpose() << std::endl;
//  std::cout << "majority_cam_source: " << majority_cam_source_ << std::endl;
//  std::cout << cam_origins.col(majority_cam_source).transpose() << std::endl;
	if (normal_.dot(source_to_sample) > 0) // normal points away from source
		normal_ *= -1.0;
	if (binormal_.dot(source_to_sample) > 0) // binormal points away from source
		binormal_ *= -1.0;

	// adjust curvature axis to new frame
	curvature_axis_ = normal_.cross(binormal_);
}

void Quadric::unpackQuadric()
{
	double a = parameters_(0);
	double b = parameters_(1);
	double c = parameters_(2);
	double d = parameters_(3);
	double e = parameters_(4);
	double f = parameters_(5);
	double g = parameters_(6);
	double h = parameters_(7);
	double i = parameters_(8);
	double j = parameters_(9);

	Eigen::Matrix3d parameter_matrix;
	parameter_matrix << a, d, f, d, b, e, f, e, c;
	Eigen::Vector3d ghi;
	ghi << g, h, i;
	Eigen::Matrix3d inverse_parameter_matrix = parameter_matrix.inverse();
	centroid_ = -0.5 * inverse_parameter_matrix * ghi;
	double k = j - 0.25 * ghi.transpose() * inverse_parameter_matrix * ghi;
	covariance_matrix_ = -1 * parameter_matrix / k;
}

bool Quadric::solveGeneralizedEigenProblem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
		Eigen::MatrixXd& v, Eigen::MatrixXd& lambda)
{
	int N = A.cols(); // Number of columns of A and B. Number of rows of v.
	if (B.cols() != N || A.rows() != N || B.rows() != N)
		return false;

	v.resize(N, N);
	lambda.resize(N, 3);

	int LDA = A.outerStride();
	int LDB = B.outerStride();
	int LDV = v.outerStride();

	double WORKDUMMY;
	int LWORK = -1; // Request optimum work size.
	int INFO = 0;

	double* alphar = const_cast<double*>(lambda.col(0).data());
	double* alphai = const_cast<double*>(lambda.col(1).data());
	double* beta = const_cast<double*>(lambda.col(2).data());

	// Get the optimum work size.
	dggev_("N", "V", &N, A.data(), &LDA, B.data(), &LDB, alphar, alphai, beta, 0, &LDV, v.data(), &LDV,
			&WORKDUMMY, &LWORK, &INFO);

	LWORK = int(WORKDUMMY) + 32;
	Eigen::VectorXd WORK(LWORK);

	dggev_("N", "V", &N, A.data(), &LDA, B.data(), &LDB, alphar, alphai, beta, 0, &LDV, v.data(), &LDV,
			WORK.data(), &LWORK, &INFO);

	return INFO == 0;
}

void Quadric::plotAxes(void* viewer_void, int id) const
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
			pcl::visualization::PCLVisualizer> *>(viewer_void);

	pcl::PointXYZ p, q, r;
	p.x = sample_(0);
	p.y = sample_(1);
	p.z = sample_(2);
	q.x = p.x + 0.02 * curvature_axis_(0);
	q.y = p.y + 0.02 * curvature_axis_(1);
	q.z = p.z + 0.02 * curvature_axis_(2);
	r.x = p.x + 0.02 * normal_(0);
	r.y = p.y + 0.02 * normal_(1);
	r.z = p.z + 0.02 * normal_(2);
//  std::cout << "p: " << p << std::endl;
//  std::cout << "q: " << q << std::endl;
//  std::cout << "r: " << r << std::endl;
	viewer->addLine<pcl::PointXYZ>(p, q, 0, 0, 255, "curvature_axis_" + boost::lexical_cast<std::string>(id));
	viewer->addLine<pcl::PointXYZ>(p, r, 255, 0, 0, "normal_axis_" + boost::lexical_cast<std::string>(id));
}
