#ifndef REPORTGEN_H
#define REPORTGEN_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "fast_global_registration.h"

using namespace std;
void generateReport(FastGlobalRegistration fgr, std::string filename){
	if(!filename.compare("")) return;
	Eigen::Matrix4f T = fgr.GetTrans();

	ofstream fid;
	fid.open (filename);

	fid<< "<html lang=\"en\">"<<endl;
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
	fid<< "<h1 align=center>Fast Global Registration </h1>"<<endl;

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

	/* RMSE plot div */
	fid<< "<h3>RMSE</h3>"<<endl;
	fid<< "<div id=\"plot\"></div>"<<endl;

	/* RMSE plot data */
	fid<< "<script>"<<endl;
	fid<< "var trace = {"<<endl;
	fid<< "\tx: [";	for (int i = 1; i < fgr.fitness.size(); i++) fid<<i<<","; fid<< fgr.fitness.size()<<"],"<<endl;
	fid<< "\ty: ["; for (int i = 0; i < fgr.fitness.size()-1; i++) fid<<fgr.fitness[i]<<","; fid<< fgr.fitness[fgr.fitness.size()-1]<<"],"<<endl;
	fid<< "\tmode: 'markers'"<<endl;
	fid<< "};"<<endl;
	fid<< "var data = [ trace ];"<<endl;
	fid<< "var layout = { title:'RMSE' };"<<endl;
	fid<< "Plotly.newPlot('plot', data, layout);"<<endl;
	fid<< "</script>"<<endl;

	/* Algorithm parameters */
	fid<< "<h3>Algorithm parameters</h3>"<<endl;
	fid<< "<div><table class=\"parameters\">"<<endl;
	fid<< "<tr><td>Closed Form</td><td>"; if(fgr.closed_form) fid<<"yes"; else fid<<"no"; fid<<"</td>"<<endl;
	fid<< "<tr><td>Scale</td><td>";	if(fgr.use_absolute_scale) fid<<"absolute"; else fid<<"relative"; fid<<"</td>"<<endl;
	fid<< "<tr><td>Div. factor</td><td>"<< fgr.div_factor <<"</td>"<<endl;
	fid<< "<tr><td>Max. corelation distance</td><td>"<< fgr.max_corr_dist<<"</td>"<<endl;
	fid<< "<tr><td>Number of iterations</td><td>"<< fgr.iteration_number <<"</td>"<<endl;
	fid<< "<tr><td>Similarity measure</td><td>"<< fgr.tuple_scale <<"</td>"<<endl;
	fid<< "<tr><td>Max. corr. tuples</td><td>"<< fgr.tuple_max_count <<"</td>"<<endl;
	fid<< "<tr><td>Normals search radius</td><td>"<< fgr.normals_search_radius <<"</td>"<<endl;
	fid<< "<tr><td>FPFH search radius</td><td>"<< fgr.fpfh_search_radius <<"</td>"<<endl;
	fid<< "</table></div>"<<endl;

	fid<< "</body>"<<endl;
	fid<< "</html>"<<endl;
	fid.close();
}

#endif