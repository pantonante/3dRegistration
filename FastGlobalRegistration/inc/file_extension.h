inline std::string get_file_extension(const std::string& filename){
	size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return "";
    return filename.substr(lastdot, std::string::npos); 
}

inline bool is_PLYfile(const std::string& filename){
	std::string extension = get_file_extension(filename);
	return (extension.compare(".ply"))?false:true;
}