#include "fgr_options.h"
#include <iostream>

int FGROptions::parse(int argc, char** argv)
{
    po::options_description desc("Fast Global Registration CLI parameters");
    desc.add_options() 
      ("help,h", "Print help messages") 
      ("pointcloudP,p",po::value<std::string>()->required(), "Point cloud filename [*.pcd]") 
      ("pointcloudQ,q",po::value<std::string>()->required(), "Point cloud filename [*.pcd]")
    ; 

    po::options_description hidden;
    // no hidden parameters

    po::options_description all_options;
    all_options.add(desc);

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
	        std::cout << make_usage_string_(basename_(argv[0]), desc, p) << '\n';
	        return FGROptions::OPTPARSE_HELP;
	    }

	    po::notify(vm);
	} catch (po::error& e) { 
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cout << make_usage_string_(basename_(argv[0]), desc, p) << '\n';
		return FGROptions::OPTPARSE_FAIL; 
	}

    ptCloudP_filename = vm["pointcloudP"].as<std::string>();
    ptCloudQ_filename = vm["pointcloudQ"].as<std::string>();

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