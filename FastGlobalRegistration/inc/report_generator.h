#ifndef REPORTGEN_H
#define REPORTGEN_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "fast_global_registration.h"

using namespace std;

void HTMLreport(FastGlobalRegistration fgr, std::string filename){
	if(!filename.compare("")) return;
	Eigen::Matrix4f T = fgr.GetTrans();
	vector<TimingInfo> ti = fgr.getTimingInfo();

	ofstream fid;
	fid.open (filename);

	fid<< "<head>"<<endl;

	/* Style (CSS) */
	fid<< "<style>"<<endl;
	fid<< ".matrix { position: relative; }"<<endl;
	fid<< "body { padding: 20px; }"<<endl;
	fid<< ".matrix:before, .matrix:after {"<<endl;
	fid<< "    content: \"\";"<<endl;
	fid<< "    position: absolute;"<<endl;
	fid<< "    top: 0;"<<endl;
	fid<< "    border: 1px solid #000;"<<endl;
	fid<< "    width: 6px;"<<endl;
	fid<< "    height: 100%;"<<endl;
	fid<< "}"<<endl;
	fid<< ".matrix:before {"<<endl;
	fid<< "    left: -6px;"<<endl;
	fid<< "    border-right: 0px;"<<endl;
	fid<< "}"<<endl;
	fid<< ".matrix:after {"<<endl;
	fid<< "    right: -6px;"<<endl;
	fid<< "    border-left: 0px;"<<endl;
	fid<< "}"<<endl;
	fid<< ".matrix td {"<<endl;
	fid<< "    padding: 5px;"<<endl; 
	fid<< "    text-align: center;"<<endl;
	fid<< "}"<<endl;
	fid<< ".parameters {"<<endl; 
	fid<< "    border-collapse: collapse;"<<endl; 
	fid<< "    width: 100%;"<<endl; 
	fid<< "}"<<endl; 
	fid<< ".parameters td, #customers th {"<<endl; 
	fid<< "    border: 1px solid #ddd;"<<endl; 
	fid<< "    padding: 8px;"<<endl; 
	fid<< "}"<<endl; 
	fid<< ".parameters tr:nth-child(even){background-color: #f2f2f2;}"<<endl; 
	fid<< ".parameters tr:hover {background-color: #ddd;}"<<endl; 
	fid<< ".parameters th {"<<endl; 
	fid<< "    padding-top: 12px;"<<endl; 
	fid<< "    padding-bottom: 12px;"<<endl; 
	fid<< "    text-align: left;"<<endl; 
	fid<< "    background-color: #4CAF50;"<<endl; 
	fid<< "    color: white;"<<endl; 
	fid<< "}"<<endl; 
	fid<< "</style>"<<endl;

	/* Plot.ly */
	fid<< "<script src=\"https://cdn.plot.ly/plotly-latest.min.js\"></script>"<<endl;

	fid<< "</head>"<<endl;
	
	/* Body */
	fid<< "<body>"<<endl;
	fid<< "<h1 align=center>Fast Global Registration</h1>"<<endl;

	/* Transformation matrix */
	fid<< "<h3>Transformation matrix</h3>"<<endl;
	fid<< "<div align=center>"<<endl;
	fid<< "    <table class=\"matrix\">"<<endl;
	fid<< "        <tr>"<<endl;
	fid<< "            <td>"<< T(0,0) <<"</td>"<<endl;
	fid<< "            <td>"<< T(0,1) <<"</td>"<<endl;
	fid<< "            <td>"<< T(0,2) <<"</td>"<<endl;
	fid<< "            <td>"<< T(0,3) <<"</td>"<<endl;
	fid<< "        </tr>"<<endl;
	fid<< "        <tr>"<<endl;
	fid<< "            <td>"<< T(1,0) <<"</td>"<<endl;
	fid<< "            <td>"<< T(1,1) <<"</td>"<<endl;
	fid<< "            <td>"<< T(1,2) <<"</td>"<<endl;
	fid<< "            <td>"<< T(1,3) <<"</td>"<<endl;
	fid<< "        </tr>"<<endl;
	fid<< "        <tr>"<<endl;
	fid<< "            <td>"<< T(2,0) <<"</td>"<<endl;
	fid<< "            <td>"<< T(2,1) <<"</td>"<<endl;
	fid<< "            <td>"<< T(2,2) <<"</td>"<<endl;
	fid<< "            <td>"<< T(2,3) <<"</td>"<<endl;
	fid<< "        </tr>"<<endl;
	fid<< "        <tr>"<<endl;
	fid<< "            <td>"<< T(3,0) <<"</td>"<<endl;
	fid<< "            <td>"<< T(3,1) <<"</td>"<<endl;
	fid<< "            <td>"<< T(3,2) <<"</td>"<<endl;
	fid<< "            <td>"<< T(3,3) <<"</td>"<<endl;
	fid<< "        </tr>"<<endl;
	fid<< "    </table>"<<endl;
	fid<< "</div>"<<endl;

	/* Timing information */
	fid<< "<h3>Timing</h3>"<<endl;
	fid<< "<div id=\"time\"><pre>" << fgr.getTiming() << "</pre></div>"<<endl;
	fid<<"<div id=\"timeplot\"></div>"<<endl; 

	/* RMSE plot div */
	fid<< "<h3>RMSE</h3>"<<endl;
	fid<< "<div id=\"rmseplot\"></div>"<<endl;

	/* Plotly data */
	fid<< "<script>"<<endl;
	fid<< "var rmse_trace = {"<<endl;
	if(fgr.fitness.size()>0){
		fid<< "\tx: [";	for (int i = 1; i < fgr.fitness.size(); i++) fid<<i<<","; fid<<fgr.fitness.size()<<"],"<<endl;
		fid<< "\ty: ["; for (int i = 0; i < fgr.fitness.size()-1; i++) fid<<sqrt(fgr.fitness[i])<<","; fid<< sqrt(fgr.fitness[fgr.fitness.size()-1])<<"],"<<endl;
	} else {
		fid<< "\tx: [],"<<endl;
		fid<< "\ty: []"<<endl;
	}
	fid<< "\tmode: 'markers'"<<endl;
	fid<< "};"<<endl;
	fid<< "var rmse_layout = { title:'RMSE', xaxis:{title: 'Iteration'}, yaxis: {title: 'RMSE'} };"<<endl;
	fid<< "Plotly.newPlot('rmseplot', [rmse_trace], rmse_layout);"<<endl;
	fid<< endl<<endl;
	fid<< "var time_data = [{"<<endl;
	fid<< "\tx: [";
	for (int i = 0; i < ti.size(); i++){
		fid<<"'"<<ti[i].first<<"'";
		if(i<ti.size()-1)
			fid<<",";
	}
	fid<<"\t],"<<endl;
	fid<< "\ty: [";
	for (int i = 0; i < ti.size(); i++){
		fid<<ti[i].second;
		if(i<ti.size()-1)
			fid<<",";
	}
	fid<< "\t],"<<endl;
	fid<< "\ttype: 'bar'"<<endl;
	fid<< "\t}];"<<endl;
	fid<< "var time_layout = {title: 'Timing information',yaxis: {title: 'Time (sec)'}};"<<endl;
	fid<< "Plotly.newPlot('timeplot', time_data, time_layout);"<<endl;
	fid<< "</script>"<<endl;

	/* Algorithm parameters */
	fid<< "<h3>Algorithm parameters</h3>"<<endl;
	fid<< "<div><table class=\"parameters\">"<<endl;
	fid<< "<tr><td>Closed Form</td><td>"; if(fgr.closed_form) fid<<"yes"; else fid<<"no"; fid<<"</td>"<<endl;
	fid<< "<tr><td>Scale</td><td>";	if(fgr.use_absolute_scale) fid<<"absolute"; else fid<<"relative"; fid<<"</td>"<<endl;
	fid<< "<tr><td>Stop RMSE</td><td>"<< sqrt(fgr.stop_mse) <<"</td>"<<endl;
	fid<< "<tr><td>Div. factor</td><td>"<< fgr.div_factor <<"</td>"<<endl;
	fid<< "<tr><td>Max. correlation distance</td><td>"<< fgr.max_corr_dist<<"</td>"<<endl;
	fid<< "<tr><td>Max. number of iterations</td><td>"<< fgr.iteration_number <<"</td>"<<endl;
	fid<< "<tr><td>Similarity measure</td><td>"<< fgr.tuple_scale <<"</td>"<<endl;
	fid<< "<tr><td>Max. corr. tuples</td><td>"<< fgr.tuple_max_count <<"</td>"<<endl;
	fid<< "<tr><td>Normals search radius</td><td>"<< fgr.normals_search_radius <<"</td>"<<endl;
	fid<< "<tr><td>FPFH search radius</td><td>"<< fgr.fpfh_search_radius <<"</td>"<<endl;
	fid<< "</table></div>"<<endl;

	fid<< "</body>"<<endl;
	fid<< "</html>"<<endl;
	fid.close();
}

void JSONreport(FastGlobalRegistration fgr, std::string filename){
	if(!filename.compare("")) return;
	Eigen::Matrix4f T = fgr.GetTrans();

	ofstream fid;
	fid.open (filename);

	fid<< "{"<<endl;
	fid<< "\"transformation\": ["<<endl;
	fid<< "\t["<<T(0,0)<<", "<<T(0,1)<<", "<<T(0,2)<<", "<<T(0,3)<<"],"<<endl;
	fid<< "\t["<<T(1,0)<<", "<<T(1,1)<<", "<<T(1,2)<<", "<<T(1,3)<<"],"<<endl;
	fid<< "\t["<<T(2,0)<<", "<<T(2,1)<<", "<<T(2,2)<<", "<<T(2,3)<<"],"<<endl;
	fid<< "\t["<<T(3,0)<<", "<<T(3,1)<<", "<<T(3,2)<<", "<<T(3,3)<<"]"<<endl;
	fid<< "],"<<endl;
	if(fgr.fitness.size()>0){
		fid<< "\"RMSE\": [";
		for (int i = 0; i < fgr.fitness.size()-1; i++) fid<<sqrt(fgr.fitness[i])<<","; fid<< sqrt(fgr.fitness[fgr.fitness.size()-1]);
		fid<< "],"<<endl;
	} else {
		fid<< "\"RMSE\": [],"<<endl;
	}
	fid<< "\"num_correspondences\":" << fgr.getNumCorrespondences()<<","<<endl;
	fid<< "\"timing\": ["<<endl;
	vector<TimingInfo> ti = fgr.getTimingInfo();
	for (int i = 0; i < ti.size(); i++){
		fid<<"\t{\n\t\t\"tag\": \""<<ti[i].first<<"\","<<endl;
		fid<<"\t\t\"time\": \""<<ti[i].second<<"\""<<endl;
		fid<<"\t}";
		if(i<ti.size()-1)
			fid<<","<<endl;
		else
			fid<<endl;
	}
	fid<< "],"<<endl;
	fid<< "\"parameters\": ["<<endl;
	fid<< "\t{\n\t\t\"name\":\"Closed Form\",\n\t\t\"value\":"; if(fgr.closed_form) fid<<"\"yes\""; else fid<<"\"no\""; fid<<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Scale\",\n\t\t\"value\":";	if(fgr.use_absolute_scale) fid<<"\"absolute\""; else fid<<"\"relative\""; fid<<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Stop RMSE\",\n\t\t\"value\":"<< sqrt(fgr.stop_mse) <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Div. factor\",\n\t\t\"value\":"<< fgr.div_factor <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Max. correlation distance\",\n\t\t\"value\":"<< fgr.max_corr_dist<<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Max. number of iterations\",\n\t\t\"value\":"<< fgr.iteration_number <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Similarity measure\",\n\t\t\"value\":"<< fgr.tuple_scale <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Max. corr. tuples\",\n\t\t\"value\":"<< fgr.tuple_max_count <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"Normals search radius\",\n\t\t\"value\":"<< fgr.normals_search_radius <<"\n\t},"<<endl;
	fid<< "\t{\n\t\t\"name\":\"FPFH search radius\",\n\t\t\"value\":"<< fgr.fpfh_search_radius <<"\n\t}"<<endl;
	fid<< "]"<<endl;
	fid<< "}"<<endl;

	fid.close();
}

#endif