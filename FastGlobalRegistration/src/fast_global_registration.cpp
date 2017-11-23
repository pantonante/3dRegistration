#define TAG "[FGR] "
#include "fast_global_registration.h"

Eigen::Matrix4f FastGlobalRegistration::performRegistration(){   
	timer_.tic();
	AdvancedMatching();
	timer_.toc("Matching");

	timer_.tic();
	if(closed_form)
		OptimizePairwise_ClosedForm(iteration_number);
	else
		OptimizePairwise(iteration_number);
	timer_.toc("Registration");
	return GetTrans();
}

FastGlobalRegistration::FastGlobalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q, FGROptions options)
{
	Points pts_P, pts_Q;
	Feature feat_P,feat_Q;

	this->verbose               = options.verbose;
	this->closed_form           = options.closed_form;
	this->use_absolute_scale    = options.use_absolute_scale;		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
	this->div_factor            = options.div_factor; 				// Division factor used for graduated non-convexity
	this->max_corr_dist         = options.max_corr_dist;			// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
	this->iteration_number 		= options.iteration_number;			// Maximum number of iteration
	this->tuple_scale 			= options.tuple_scale;				// Similarity measure used for tuples of feature points.
	this->tuple_max_count 		= options.tuple_max_count;			// Maximum tuple numbers.
	this->stop_mse              = options.stop_mse;					// Stop criteria
	this->normals_search_radius = options.normals_search_radius;	// Normals estimation search radius
	this->fpfh_search_radius    = options.fpfh_search_radius;		// FPFH estimation search radius

	/* Point Clouds Initialization */
	// Point Cloud P
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = ptCloud_P->begin(); it != ptCloud_P->end(); it++){ 
		Vector3f pt;
    	pt << it->x, it->y, it->z;
    	pts_P.push_back(pt);
    }	
	pointcloud_.push_back(pts_P);

	// Point Cloud Q
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = ptCloud_Q->begin(); it != ptCloud_Q->end(); it++){ 
		Vector3f pt;
    	pt << it->x, it->y, it->z;
    	pts_Q.push_back(pt);
    }	
	pointcloud_.push_back(pts_Q);

	/* Normalization */
	timer_.tic();	
	NormalizePoints(); // don't skip this passage before computing FPFH! It compute the GlobalScale required to set correctly the search radius
	timer_.toc("Normalization");

	/* Features */
	timer_.tic();
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_P = computeFPFH(ptCloud_P);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_Q = computeFPFH(ptCloud_Q);
    timer_.toc("Features computation");

    //data structure filling
    for(pcl::PointCloud<pcl::FPFHSignature33>::iterator it = fpfh_P->begin(); it != fpfh_P->end(); it++){ 
    	VectorXf fpfh(33);
    	for(int i=0;i<33;i++)
    		fpfh(i) = it->histogram[i];
    	feat_P.push_back(fpfh);
    }
    features_.push_back(feat_P);
    for(pcl::PointCloud<pcl::FPFHSignature33>::iterator it = fpfh_Q->begin(); it != fpfh_Q->end(); it++){ 
    	VectorXf fpfh(33);
    	for(int i=0;i<33;i++)
    		fpfh(i) = it->histogram[i];
    	feat_Q.push_back(fpfh);
    }
    features_.push_back(feat_Q);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr FastGlobalRegistration::computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	// Compute the normals
#ifdef USE_OMP
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
#else
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
#endif
	normalEstimation.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
	normalEstimation.setRadiusSearch (normals_search_radius);
	normalEstimation.compute (*cloudWithNormals);

	// Setup the feature computation
#ifdef USE_OMP
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
#else
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
#endif
	// Provide the original point cloud (without normals)
	fpfhEstimation.setInputCloud (cloud);
	// Provide the point cloud with normals
	fpfhEstimation.setInputNormals(cloudWithNormals);
	// fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
	// Use the same KdTree from the normal estimation
	fpfhEstimation.setSearchMethod (tree);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfhEstimation.setRadiusSearch (fpfh_search_radius);
	// Actually compute the spin images
	fpfhEstimation.compute (*pfhFeatures);

	return pfhFeatures;
}

void FastGlobalRegistration::SearchFLANNTree(flann::Index<flann::L2<float>>* index,
							VectorXf& input,
							std::vector<int>& indices,
							std::vector<float>& dists,
							int nn)
{
	int rows_t = 1;
	int dim = input.size();

	std::vector<float> query;
	query.resize(rows_t*dim);
	for (int i = 0; i < dim; i++)
		query[i] = input(i);
	flann::Matrix<float> query_mat(&query[0], rows_t, dim);

	indices.resize(rows_t*nn);
	dists.resize(rows_t*nn);
	flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
	flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

	index->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(FLANN_SEARCH_CHECK));
}

void FastGlobalRegistration::AdvancedMatching()
{
	int fi = 0;
	int fj = 1;

	if(verbose)
		cout<<TAG << "Advanced matching : ["<< fi <<" - "<< fj <<"]" << endl;

	bool swapped = false;

	if (pointcloud_[fj].size() > pointcloud_[fi].size())
	{
		int temp = fi;
		fi = fj;
		fj = temp;
		swapped = true;
	}

	int nPti = pointcloud_[fi].size();
	int nPtj = pointcloud_[fj].size();

	///////////////////////////
	/// BUILD FLANNTREE
	///////////////////////////

	// build FLANNTree - fi
	int rows, dim;
	rows = features_[fi].size();
	dim = features_[fi][0].size();

	std::vector<float> dataset_fi(rows * dim);
	flann::Matrix<float> dataset_mat_fi(&dataset_fi[0], rows, dim);

	for (int i = 0; i < rows; i++)
		for (int j = 0; j < dim; j++)
			dataset_fi[i * dim + j] = features_[fi][i][j];

	flann::Index<flann::L2<float>> feature_tree_i(dataset_mat_fi, flann::KDTreeSingleIndexParams(FLANN_LEAF_MAX_SIZE));
	feature_tree_i.buildIndex();

	// build FLANNTree - fj
	rows = features_[fj].size();
	dim = features_[fj][0].size();

	std::vector<float> dataset_fj(rows * dim);
	flann::Matrix<float> dataset_mat_fj(&dataset_fj[0], rows, dim);

	for (int i = 0; i < rows; i++)
		for (int j = 0; j < dim; j++)
			dataset_fj[i * dim + j] = features_[fj][i][j];

	flann::Index<flann::L2<float>> feature_tree_j(dataset_mat_fj, flann::KDTreeSingleIndexParams(FLANN_LEAF_MAX_SIZE));
	feature_tree_j.buildIndex();

	bool crosscheck = true;
	bool tuple = true;

	std::vector<int> corres_K, corres_K2;
	std::vector<float> dis;
	std::vector<int> ind;

	std::vector<std::pair<int, int>> corres;
	std::vector<std::pair<int, int>> corres_cross;
	std::vector<std::pair<int, int>> corres_ij;
	std::vector<std::pair<int, int>> corres_ji;

	///////////////////////////
	/// INITIAL MATCHING
	///////////////////////////

	std::vector<int> i_to_j(nPti, -1);
	for (int j = 0; j < nPtj; j++)
	{
		SearchFLANNTree(&feature_tree_i, features_[fj][j], corres_K, dis, 1);
		int i = corres_K[0];
		if (i_to_j[i] == -1)
		{
			SearchFLANNTree(&feature_tree_j, features_[fi][i], corres_K, dis, 1);
			int ij = corres_K[0];
			i_to_j[i] = ij;
		}
		corres_ji.push_back(std::pair<int, int>(i, j));
	}

	for (int i = 0; i < nPti; i++)
	{
		if (i_to_j[i] != -1)
			corres_ij.push_back(std::pair<int, int>(i, i_to_j[i]));
	}

	int ncorres_ij = corres_ij.size();
	int ncorres_ji = corres_ji.size();

	// corres = corres_ij + corres_ji;
	for (int i = 0; i < ncorres_ij; ++i)
		corres.push_back(std::pair<int, int>(corres_ij[i].first, corres_ij[i].second));
	for (int j = 0; j < ncorres_ji; ++j)
		corres.push_back(std::pair<int, int>(corres_ji[j].first, corres_ji[j].second));

	if(verbose)
		cout<<TAG<<"Correspondence computation on remained: "<<  (int)corres.size() <<endl;

	///////////////////////////
	/// CROSS CHECK
	/// input : corres_ij, corres_ji
	/// output : corres
	///////////////////////////
	if (crosscheck)
	{
		if(verbose)
			cout<<TAG<<"\t- cross check, ";

		// build data structure for cross check
		corres.clear();
		corres_cross.clear();
		std::vector<std::vector<int>> Mi(nPti);
		std::vector<std::vector<int>> Mj(nPtj);

		int ci, cj;
		for (int i = 0; i < ncorres_ij; ++i)
		{
			ci = corres_ij[i].first;
			cj = corres_ij[i].second;
			Mi[ci].push_back(cj);
		}
		for (int j = 0; j < ncorres_ji; ++j)
		{
			ci = corres_ji[j].first;
			cj = corres_ji[j].second;
			Mj[cj].push_back(ci);
		}

		// cross check
		for (int i = 0; i < nPti; ++i)
		{
			for (int ii = 0; ii < Mi[i].size(); ++ii)
			{
				int j = Mi[i][ii];
				for (int jj = 0; jj < Mj[j].size(); ++jj)
				{
					if (Mj[j][jj] == i)
					{
						corres.push_back(std::pair<int, int>(i, j));
						corres_cross.push_back(std::pair<int, int>(i, j));
					}
				}
			}
		}
		if(verbose)
			cout << "points remained: " << (int)corres.size() << endl;
	}

	///////////////////////////
	/// TUPLE CONSTRAINT
	/// input : corres
	/// output : corres
	///////////////////////////
	if (tuple)
	{
		srand(time(NULL));
		if(verbose)
			cout<<TAG<<"\t- tuple constraint, ";
		int rand0, rand1, rand2;
		int idi0, idi1, idi2;
		int idj0, idj1, idj2;
		float scale = tuple_scale;
		int ncorr = corres.size();
		int number_of_trial = ncorr * 100;
		std::vector<std::pair<int, int>> corres_tuple;

		int cnt = 0;
		int i;
		for (i = 0; i < number_of_trial; i++)
		{
			rand0 = rand() % ncorr;
			rand1 = rand() % ncorr;
			rand2 = rand() % ncorr;

			idi0 = corres[rand0].first;
			idj0 = corres[rand0].second;
			idi1 = corres[rand1].first;
			idj1 = corres[rand1].second;
			idi2 = corres[rand2].first;
			idj2 = corres[rand2].second;

			// collect 3 points from i-th fragment
			Eigen::Vector3f pti0 = pointcloud_[fi][idi0];
			Eigen::Vector3f pti1 = pointcloud_[fi][idi1];
			Eigen::Vector3f pti2 = pointcloud_[fi][idi2];

			float li0 = (pti0 - pti1).norm();
			float li1 = (pti1 - pti2).norm();
			float li2 = (pti2 - pti0).norm();

			// collect 3 points from j-th fragment
			Eigen::Vector3f ptj0 = pointcloud_[fj][idj0];
			Eigen::Vector3f ptj1 = pointcloud_[fj][idj1];
			Eigen::Vector3f ptj2 = pointcloud_[fj][idj2];

			float lj0 = (ptj0 - ptj1).norm();
			float lj1 = (ptj1 - ptj2).norm();
			float lj2 = (ptj2 - ptj0).norm();

			if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
				(li1 * scale < lj1) && (lj1 < li1 / scale) &&
				(li2 * scale < lj2) && (lj2 < li2 / scale))
			{
				corres_tuple.push_back(std::pair<int, int>(idi0, idj0));
				corres_tuple.push_back(std::pair<int, int>(idi1, idj1));
				corres_tuple.push_back(std::pair<int, int>(idi2, idj2));
				cnt++;
			}

			if (cnt >= tuple_max_count)
				break;
		}
		if(verbose)
			cout<<"tuples remained "<< cnt << " ("<< number_of_trial <<" trial, "<<i <<" actual)."<<endl;
		corres.clear();

		for (int i = 0; i < corres_tuple.size(); ++i)
			corres.push_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
	}

	if (swapped)
	{
		std::vector<std::pair<int, int>> temp;
		for (int i = 0; i < corres.size(); i++)
			temp.push_back(std::pair<int, int>(corres[i].second, corres[i].first));
		corres.clear();
		corres = temp;
	}

	corres_ = corres;

	if(verbose)
		cout<<TAG<<"\t- final matches "<< (int)corres.size() <<" tuples."<<endl;
}

// Normalize scale of points.
// X' = (X-\mu)/scale
void FastGlobalRegistration::NormalizePoints()
{
	int num = 2;
	float scale = 0;

	Means.clear();

	for (int i = 0; i < num; ++i)
	{
		float max_scale = 0;

		// compute mean
		Vector3f mean;
		mean.setZero();

		int npti = pointcloud_[i].size();
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			mean = mean + p;
		}
		mean = mean / npti;
		Means.push_back(mean);

		if(verbose)
			cout << TAG <<"Normalize points, mean(" << i << ") = ["
			<< mean(0) << " " << mean(1) << " " << mean(2) << "]" << endl;

		for (int ii = 0; ii < npti; ++ii)
		{
			pointcloud_[i][ii](0) -= mean(0);
			pointcloud_[i][ii](1) -= mean(1);
			pointcloud_[i][ii](2) -= mean(2);
		}

		// compute scale
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			float temp = p.norm(); // because we extract mean in the previous stage.
			if (temp > max_scale)
				max_scale = temp;
		}

		if (max_scale > scale)
			scale = max_scale;
	}

	//// mean of the scale variation
	if (use_absolute_scale) {
		GlobalScale = 1.0f;
		StartScale = scale;
	} else {
		GlobalScale = scale; // second choice: we keep the maximum scale.
		StartScale = 1.0f;
	}
	if(verbose)
		cout << TAG << "Normalize points, global scale: " <<  GlobalScale << endl;

	if(GlobalScale!=1.0f){
		for (int i = 0; i < num; ++i)
		{
			int npti = pointcloud_[i].size();
			for (int ii = 0; ii < npti; ++ii)
			{
				pointcloud_[i][ii](0) /= GlobalScale;
				pointcloud_[i][ii](1) /= GlobalScale;
				pointcloud_[i][ii](2) /= GlobalScale;
			}
		}
	}
}

double FastGlobalRegistration::OptimizePairwise(int numIter_)
{
	if(verbose)
		cout<<TAG<<"Pairwise rigid pose optimization (iterative)"<<endl;

	double par;
	int numIter = numIter_;
	vector<float> align_error;

	TransOutput_ = Eigen::Matrix4f::Identity();
	par = StartScale;

	int i = 0;
	int j = 1;

	// make another copy of pointcloud_[j] because we need to modify it
	Points pcj_copy;
	int npcj = pointcloud_[j].size();
	pcj_copy.resize(npcj);
	for (int cnt = 0; cnt < npcj; cnt++)
		pcj_copy[cnt] = pointcloud_[j][cnt];

	if (corres_.size() < MIN_NUM_OF_CORR)
		return -1;

	std::vector<double> s(corres_.size(), 1.0);

	Eigen::Matrix4f trans;
	trans.setIdentity();

	for (int itr = 0; itr < numIter; itr++) {
        /* Stopping conditions */
        // 1. If we achieved the right accuracy
        if(itr > 0 && fitness[itr-1]<=stop_mse)
            break;
        // 2. If the registration error is increasing
        if(itr >= MAX_INCREASING_MSE_ITERATIONS){
            bool stop = true;
            for(int i=itr-1; i>=itr-MAX_INCREASING_MSE_ITERATIONS; i--)
                stop&=(fitness[i]-fitness[i-1])>MAX_MSE_THRESHOLD;
            if(stop) break;
        }

		// graduated non-convexity.
		if (itr % 4 == 0 && par > max_corr_dist){
			par /= div_factor;
		}

		const int nvariable = 6;	// 3 for rotation and 3 for translation
		Eigen::MatrixXd JTJ(nvariable, nvariable);
		Eigen::MatrixXd JTr(nvariable, 1);
		Eigen::MatrixXd J(nvariable, 1);
		JTJ.setZero();
		JTr.setZero();

		double r;
		double r2 = 0.0;

		for (int c = 0; c < corres_.size(); c++) {
			int ii = corres_[c].first;
			int jj = corres_[c].second;
			Eigen::Vector3f p, q;
			p = pointcloud_[i][ii];
			q = pcj_copy[jj];
			Eigen::Vector3f rpq = p - q;
			align_error.push_back(rpq.dot(rpq)*GlobalScale*GlobalScale);

			int c2 = c;

			float temp = par / (rpq.dot(rpq) + par);
			s[c2] = temp * temp;

			J.setZero();
			J(1) = -q(2);
			J(2) = q(1);
			J(3) = -1;
			r = rpq(0);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			J.setZero();
			J(2) = -q(0);
			J(0) = q(2);
			J(4) = -1;
			r = rpq(1);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			J.setZero();
			J(0) = -q(1);
			J(1) = q(0);
			J(5) = -1;
			r = rpq(2);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			r2 += (par * (1.0 - sqrt(s[c2])) * (1.0 - sqrt(s[c2])));
		}

		fitness.push_back(accumulate(align_error.begin(), align_error.end(), 0.0)/align_error.size());

		Eigen::MatrixXd result(nvariable, 1);
		result = -JTJ.llt().solve(JTr);

		Eigen::Affine3d aff_mat;
		aff_mat.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX());
		aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));

		Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

		trans = delta * trans;

		// transform point clouds
		Matrix3f R = delta.block<3, 3>(0, 0);
		Vector3f t = delta.block<3, 1>(0, 3);
		for (int cnt = 0; cnt < npcj; cnt++)
			pcj_copy[cnt] = R * pcj_copy[cnt] + t;

	}

	TransOutput_ = trans * TransOutput_;
	return par;
}

//http://jackhunt.uk/github%20repositories/3d%20registration/2017/08/24/horns-method.html
//http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
double FastGlobalRegistration::OptimizePairwise_ClosedForm(int numIter_)
{
	if(verbose)
		cout<<TAG<<"Pairwise rigid pose optimization (closed form)"<<endl;

	double par;
	int numIter = numIter_;
	TransOutput_ = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
	fitness.clear();

	par = StartScale;

	int i = 0; //ptCloud_P
	int j = 1; //ptCloud_Q

	// make another copy of pointcloud_[j].
	Points pcj_copy;
	int npcj = pointcloud_[j].size();
	pcj_copy.resize(npcj);
	for (int cnt = 0; cnt < npcj; cnt++)
		pcj_copy[cnt] = pointcloud_[j][cnt];

	if (corres_.size() < 10)
		return -1;

	//std::vector<double> s(corres_.size(), 1.0);

	for (int itr = 0; itr < numIter && par > max_corr_dist; itr++)
	{
		Points p_corr, q_corr;
		vector<float> align_error;

        /* Stopping conditions */
        // 1. If we achieved the right accuracy
		if(itr > 0 && fitness[itr-1]<=stop_mse)
			break;
        // 2. If the registration error is increasing
        if(itr>=MAX_INCREASING_MSE_ITERATIONS){
            bool stop = true;
            for(int i=itr-1; i>=itr-MAX_INCREASING_MSE_ITERATIONS; i--)
                stop&=(fitness[i]-fitness[i-1])>MAX_MSE_THRESHOLD;
            if(stop) break;
        }
        
		// graduated non-convexity.
		/*if (par > max_corr_dist) {
			par /= div_factor;
		}else{
			break;
		}*/
        if(itr > 0) par /= div_factor;

		for (int c = 0; c < corres_.size(); c++) {
			int ii = corres_[c].first;
			int jj = corres_[c].second;
			Eigen::Vector3f p, q;
			p = pointcloud_[i][ii];
			q = pcj_copy[jj];
			Eigen::Vector3f rpq = p - q;
			align_error.push_back(rpq.dot(rpq)*GlobalScale*GlobalScale);

			float l_pq = par / (rpq.dot(rpq) + par);
			float s = sqrt(l_pq);
			p_corr.push_back(p*s);
			q_corr.push_back(q*s);
		}
		fitness.push_back(accumulate(align_error.begin(), align_error.end(), 0.0)/align_error.size());

		// Find the transformation matrix (closed form)
		Map<Matrix3Xf> ps(&p_corr[0].x(),3,corres_.size());
		Map<Matrix3Xf> qs(&q_corr[0].x(),3,corres_.size());		
		Matrix3f K = ps * qs.transpose();
	    JacobiSVD<Matrix3f> svd(K, ComputeFullU | ComputeFullV);
	    Matrix3f R = svd.matrixU()*svd.matrixV().transpose();
	    if(R.determinant()<0) //this is to account for a numerical issue that may occur, causing the rotation to be reflected.
	        R.col(2) *= -1;
	    Vector3f t = ps.rowwise().mean() - R*qs.rowwise().mean();

	    // Update with the last matrix
	    T.block<3, 3>(0, 0) = R;
		T.block<3, 1>(0, 3) = t;
		TransOutput_ = T * TransOutput_;

		// transform point clouds
		for (int cnt = 0; cnt < npcj; cnt++)
			pcj_copy[cnt] = R * pcj_copy[cnt] + t;

	}
	return par;
}

Eigen::Matrix4f FastGlobalRegistration::GetTrans()
{
    Eigen::Matrix3f R;
	Eigen::Vector3f t;
	R = TransOutput_.block<3, 3>(0, 0);
	t = TransOutput_.block<3, 1>(0, 3);

	Eigen::Matrix4f transtemp = Eigen::Matrix4f::Identity();;
	transtemp.block<3, 3>(0, 0) = R;
	transtemp.block<3, 1>(0, 3) = -R*Means[1] + t*GlobalScale + Means[0];
    
    return transtemp;
}
