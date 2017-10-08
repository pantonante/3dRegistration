#include <iostream>
#include "fgr_options.h"
#include "default_config.h"

int FGROptions::parse(int argc, char** argv)
{
    po::options_description desc("Required arguments");
    desc.add_options()       
      ("pointcloudP,p",         po::value<std::string>(&ptCloudP_filename)->required(), 
        "Point cloud filename [*.pcd].\n") 
      ("pointcloudQ,q",         po::value<std::string>(&ptCloudQ_filename)->required(), 
        "Point cloud filename [*.pcd].\n")
    ; 

    po::options_description misc("Miscellaneous");
    misc.add_options()
      ("help,h", "Print help messages\n")
      ("verbose,v",             po::bool_switch(&verbose)->default_value(VERBOSE), 
        "Verbose output.\n")
      ("output,o",              po::value<std::string>(&outputfile)->default_value(""),
        "Output filename, save the transformation matrix.\n")
      ("fitness,f",              po::value<std::string>(&fitnessfile)->default_value(""),
        "Save to file the RMSE in each iteration.\n")
      ("report,r",              po::value<std::string>(&reportfile)->default_value(""),
        "Save an HTML report.\n")
    ;

    po::options_description algpar("Algorithm parameters");
    algpar.add_options()
      ("abs-scale,a",           po::bool_switch(&use_absolute_scale)->default_value(USE_ABSOLUTE_SCALE), 
        "If enabled, measure distance in absolute scale, otherwise in scale relative to the diameter of the model.\n")
      ("closed-form,c",         po::bool_switch(&closed_form)->default_value(CLOSED_FORM),
        "Use closed form solution for transformation estimation.\n")
      ("div-factor",            po::value<float>(&div_factor)->default_value(DIV_FACTOR), 
        "Division factor used for graduated non-convexity.\n")
      ("stop-rmse",            po::value<float>(&stop_mse)->default_value(STOP_RMSE), 
        "Optimization stops when reach the given RMSE.\n")
      ("max-corr-dist",         po::value<float>(&max_corr_dist)->default_value(MAX_CORR_DIST), 
        "Maximum correspondence distance (also see abs-scale).\n")
      ("iterations,n",          po::value<int>(&iteration_number)->default_value(ITERATION_NUMBER),
        "Maximum number of iteration.\n")
      ("tuple-scale",           po::value<float>(&tuple_scale)->default_value(TUPLE_SCALE),
        "Similarity measure used for tuples of feature points.\n")
      ("tuple-max-count,m",       po::value<int>(&tuple_max_count)->default_value(TUPLE_MAX_CNT),
        "Maximum tuple numbers.\n")
      ("normals-search-radius", po::value<float>(&normals_search_radius)->default_value(NORMALS_SEARCH_RAD),
        "Normals estimation search radius.\n")
      ("fpfh-search-radius",    po::value<float>(&fpfh_search_radius)->default_value(FPFH_SEARCH_RAD),
        "FPFH estimation search radius.\n")
    ;

    po::options_description hidden;
    // no hidden parameters

    po::options_description all_options;
    all_options.add(desc);
    all_options.add(misc);
    all_options.add(algpar);

    po::positional_options_description p;
    //no positional parameters

    po::variables_map vm;
    try{
	    po::store(po::command_line_parser(argc, argv).
	            options(all_options).
	            positional(p).
	            run(),
	            vm);

	    if(vm.count("help"))
	    {
	        std::cout << make_usage_string_(basename_(argv[0]), all_options, p) << '\n';
	        return FGROptions::OPTPARSE_HELP;
	    }

	    po::notify(vm);

        stop_mse = stop_mse*stop_mse; // the user puts the RMSE, we need the MSE

	} catch (po::error& e) { 
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cout << make_usage_string_(basename_(argv[0]), all_options, p) << '\n';
		return FGROptions::OPTPARSE_FAIL; 
	}

    return FGROptions::OPTPARSE_SUCCESS;
}

/*********************************************************
* 				Private functions						 *
*********************************************************/
std::string FGROptions::basename_(const std::string& p)
{
#ifdef HAVE_BOOST_FILESYSTEM
    return boost::filesystem::path(p).stem().string();
#else
    size_t start = p.find_last_of("/");
    if(start == std::string::npos)
        start = 0;
    else
        ++start;
    return p.substr(start);
#endif
}

// Boost doesn't offer any obvious way to construct a usage string
// from an infinite list of positional parameters.  This hack
// should work in most reasonable cases.
std::vector<std::string> FGROptions::get_unlimited_positional_args_(const po::positional_options_description& p)
{
    assert(p.max_total_count() == std::numeric_limits<unsigned>::max());

    std::vector<std::string> parts;

    // reasonable upper limit for number of positional options:
    const int MAX = 1000; 
    std::string last = p.name_for_position(MAX);

    for(size_t i = 0; true; ++i)
    {
        std::string cur = p.name_for_position(i);
        if(cur == last)
        {
            parts.push_back(cur);
            parts.push_back('[' + cur + ']');
            parts.push_back("...");
            return parts;
        }
        parts.push_back(cur);
    }
    return parts; // never get here
}

std::string FGROptions::make_usage_string_(const std::string& program_name, const po::options_description& desc, po::positional_options_description& p)
{
    std::vector<std::string> parts;
    parts.push_back("Usage: ");
    parts.push_back(program_name);
    size_t N = p.max_total_count();
    if(N == std::numeric_limits<unsigned>::max())
    {
        std::vector<std::string> args = get_unlimited_positional_args_(p);
        parts.insert(parts.end(), args.begin(), args.end());
    }
    else
    {
        for(size_t i = 0; i < N; ++i)
        {
            parts.push_back(p.name_for_position(i));
        }
    }
    if(desc.options().size() > 0)
    {
        parts.push_back("[options]");
    }
    std::ostringstream oss;
    std::copy(
            parts.begin(),
            parts.end(),
            std::ostream_iterator<std::string>(oss, " "));
    oss << '\n' << desc;
    return oss.str();
}