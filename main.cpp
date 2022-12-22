#include <unordered_map>
#include <vector>
#include <string>
#include <tuple>
#include <iostream>
#include <fstream>
#include <string_view>
#include <optional>
#include <iomanip>
#include <cmath>
#include <set>
#include "json.h"
#include "graph.h"
#include "router.h"
//#include "profile.h"

using namespace std;
using DistList = unordered_map<string, double>;
using RouteInstruction = tuple<string,string,double, int>;
using RouteDescription = tuple<int, double, vector<RouteInstruction>>;
const static double Pi = 3.1415926535;

enum RTYPE{
	BUS,
	STOP,
	ROUTE
};

pair<string_view, optional<string_view>> SplitTwoStrict(string_view s, string_view delimiter = " ") {
	const size_t pos = s.find(delimiter);
	if (pos == s.npos) {
		return {s, nullopt};
	} else {
		return {s.substr(0, pos), s.substr(pos + delimiter.length())};
	}
}

pair<string_view, string_view> SplitTwo(string_view s, string_view delimiter = " ") {
	const auto [lhs, rhs_opt] = SplitTwoStrict(s, delimiter);
	return {lhs, rhs_opt.value_or("")};
}

string_view ReadToken(string_view& s, string_view delimiter = " ") {
	const auto [lhs, rhs] = SplitTwo(s, delimiter);
	s = rhs;
	return lhs;
}

template <typename Number>
Number ReadNumberOnLine(istream& stream) {
	Number number;
	stream >> number;
	string dummy;
	getline(stream, dummy);
	return number;
}

struct StopInfo{
	StopInfo() = default;
	int id = -1;
	double lat = 0;
	double lon = 0;
	vector<pair<string, double>> dists;
	StopInfo& operator*=(double factor){
		lat *= factor;
		lon *= factor;
		return *this;
	}

};

class TransportManager{
public:
	TransportManager() = default;

	void SetVelocityWait(double velocity, int wait){ velocity_ = velocity; wait_= wait;}

	void AddStop(int id, string stop_name, double lat, double lon, vector<pair<string, double>> dists){
		for (const auto& [stop,dist] : dists){
			stop_dists_[stop_name][stop] = dist;

			if (!stop_dists_[stop].count(stop_name)){
				stop_dists_[stop][stop_name] = dist;
			}
		}
		stop_by_id_[id] = stop_name;
		stops_[stop_name] = {id, lat,lon, move(dists)};
	}

	void AddBus(string bus_name, vector<string> stops, bool is_circle){
		is_bus_circle_[bus_name] = is_circle;
		bus_routes_[bus_name] = move(stops);
	}

	void AddRequest(RTYPE req_type,int id,string from,string to){
		requests_.push_back(make_tuple(req_type, id, from, to));
	}

	void CreateReqDB() const{
		StopReqFill();
		BusReqFill();
		RouteReqFill();
	}

	void ProcessRequests (ostream& out = cerr) const{
		Graph::Router<double> router_ (route_req_graph_);

		out << "[" << endl;
		size_t count = 0;
		for (const auto& [type, id, name1, name2]: requests_){
			out << "{" << endl;

			if (type == BUS){
				if (bus_req_database.count(name1)){
					auto answer = bus_req_database.at(name1);
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"stop_count\"" << ": " << get<0>(answer) << "," << endl;
					out << "\"unique_stop_count\"" << ": " << get<1>(answer) << "," << endl;
					out << "\"route_length\"" << ": " << get<2>(answer)<< "," << endl;
					out << "\"curvature\"" << ": " << get<3>(answer) << endl;
				}
				else{
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"error_message\"" << ": " << "\"not found\"" << endl;
				}
			}

			else if (type == STOP){
				if (stops_.count(name1)){
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"buses\"" << ": " << "[" << endl;
					auto set_ref = stop_req_database[name1];
					size_t count = 0;
					for(const auto& bus : set_ref){
						out << "\"" << bus << "\"";
						count++;
						if (count < set_ref.size()){
							out << "," << endl;
						}
					}

					out << endl << "]" << endl;
				}
				else{
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"error_message\"" << ": " << "\"not found\"" << endl;
				}
			}

			else if(type == ROUTE){
				Graph::VertexId vertex_from = stops_.at(name1).id; // inlet of the start station - wait for a bus first
				Graph::VertexId vertex_to = stops_.at(name2).id ; //  inlet of the destination station
				auto route_info = router_.BuildRoute(vertex_from, vertex_to);
				if (route_info != nullopt){
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"total_time\"" << ": " << route_info.value().weight << "," << endl;
					if (name1 != name2){
						out << "\"items\"" << ": " << "[" << endl;
						size_t items_count = 0;
						for (int i = 0; i < route_info.value().edge_count; i++){
							auto edge_id = router_.GetRouteEdge(route_info.value().id,i);
							auto act  = instructions_[edge_id];

							out << "{" << endl;
							if (get<0>(act) == "Wait"){
								out << "\"type\"" << ": \"" << get<0>(act) << "\" ," << endl;
								out << "\"stop_name\"" <<": \"" << get<1>(act) << "\" ," << endl;
								out << "\"time\"" << ": " << get<2>(act) << endl;
							}
							else if (get<0>(act) == "Bus"){
								out << "\"type\"" << ": \"" << get<0>(act) << "\" ," << endl;
								out << "\"bus\"" << ": \"" << get<1>(act) << "\" ," << endl;
								out << "\"span_count\"" << ": " << get<3>(act) << "," << endl;
								out << "\"time\"" << ": " << get<2>(act) << endl;
							}
							else {throw invalid_argument("invalid type in items");}
							out << "}";
							items_count++;
							if (items_count < (route_info.value().edge_count)){
								out << ",";
							}
							out << endl;
						}
						out << "]" << endl;
					}
					else{
						out << "\"items\": []"<< endl;
					}
				}
				else{
					out << "\"request_id\"" << ": " << id << "," << endl;
					out << "\"error_message\"" << ": " << "\"not found\"" << endl;
				}
			}

			out << "}";
			count++;
			if (count < requests_.size()){
				out << ",";
			}
			out << endl;
		}
		out << "]" << endl;
	}

	void PrintInstructions(){
		cout << endl << "#######################" << endl << "Instructions: " << endl;
		for (const auto& [id, act] : instructions_){
			cout << endl;
			cout << "Edge id " << id << endl;
			cout << "\"type\"" << ": \"" << get<0>(act) << "\" ," << endl;
			cout << "\"name\"" << ": \"" << get<1>(act) << "\" ," << endl;
			cout << "\"span_count\"" << ": " << get<3>(act) << "," << endl;
			cout << "\"time\"" << ": " << get<2>(act) << endl;
		}
		cout << endl << "######################" << endl << endl;
	}


private:

	double ComputeDistance(string stop1, string stop2) const{
		StopInfo lhs(stops_.at(stop1));
		StopInfo rhs(stops_.at(stop2));
		lhs *= Pi / 180.0;
		rhs *= Pi / 180.0;
		double res = acos(sin(lhs.lat) * sin(rhs.lat) +
				cos(lhs.lat) * cos(rhs.lat) *
				cos(abs(lhs.lon - rhs.lon)) ) * 6371000;
		return res;
	}

	void StopReqFill() const{
		for (const auto& [bus_name,stops] : bus_routes_){
			for (const auto& stop : stops){
				stop_req_database[stop].insert(bus_name);
			}
		}
	}

	void BusReqFill() const{
		for (const auto& [bus_name, stops]: bus_routes_){

			int all_stops = 0;
			int unique_stops = 0;
			double length = 0;
			uint real_dist = 0;
			double curvature = 0;

			if (stops.size() == 0){
				bus_req_database[bus_name] = make_tuple(all_stops,unique_stops,real_dist, curvature);
				continue;
			}

			if (is_bus_circle_.at(bus_name)){
				all_stops = stops.size();

			}
			else {
				all_stops  = stops.size() * 2 - 1;
			}

			// unique_stops
			auto temp = stops;
			sort(temp.begin(),temp.end());
			unique_stops = unique(temp.begin(), temp.end()) - temp.begin();

			// computing length and real distances
			auto it1 = stops.begin();
			while (it1 != stops.end()){
				auto it2 = it1 + 1;
				if (it2 != stops.end()){
					if (is_bus_circle_.at(bus_name)){
						length += ComputeDistance(*it1,*it2);
						real_dist += stop_dists_.at(*it1).at(*it2);
					}
					else{
						length += 2 * ComputeDistance(*it1, *it2);
						real_dist += stop_dists_.at(*it1).at(*it2);
						real_dist += stop_dists_.at(*it2).at(*it1);
					}
				}
				it1++;
			}
			curvature = real_dist / length;
			bus_req_database[bus_name] = make_tuple(all_stops,unique_stops,real_dist, curvature);
		}
	}

	void RouteReqFill() const{
		// adding wait edges, every bus stop  has inlet and outlet vertexes
		route_req_graph_ = Graph::DirectedWeightedGraph<double>(stops_.size() * 2);
		size_t pos_count = 0;
		while (pos_count < route_req_graph_.GetVertexCount()){
			Graph::EdgeId id = route_req_graph_.AddEdge({pos_count, pos_count + 1, static_cast<double>(wait_)}); // wait edges
			instructions_[id] = {"Wait",stop_by_id_.at(pos_count) , wait_, 0};
			pos_count += 2;
		}
		// adding bus routes
		for (const auto& [bus_name, bus_stops] : bus_routes_){

			if(is_bus_circle_.at(bus_name)){
				if (bus_stops.size() < 3){throw invalid_argument("RouteReqFill invalid circle route");}
				int step = bus_stops.size() - 1;

				for(int i = 0; i < step; i++){
					double weight_count = 0;
					int span_count = 0;
					auto it_start = bus_stops.begin() + i;
					auto it_delta = it_start;

					for (int j = 0; j < step - i; j++){
						it_delta++;
						string from = *it_start;
						string to = *it_delta;
						string point = *(it_delta - 1);
						if (!(to == *(bus_stops.begin() + i))){
							Graph::VertexId vertex_from = stops_.at(from).id + 1; // odd outlet
							Graph::VertexId vertex_to = stops_.at(to).id ; //  even inlet
							weight_count += static_cast<double> (stop_dists_.at(point).at(to)) /(velocity_ * 1000 / 60 );
							span_count++;
							Graph::EdgeId id = route_req_graph_.AddEdge({vertex_from, vertex_to, weight_count});
							instructions_[id] = {"Bus", bus_name,weight_count, span_count};
//							cout << endl << "Adding instruction   " << from << "  to  "	<< to << endl;
//							cout << "span count is " << span_count << "  and weight is " << weight_count << endl << endl;
						}
						else{
							weight_count += static_cast<double> (stop_dists_.at(point).at(to)) /(velocity_ * 1000 / 60 );
							span_count++;
						}
					}
				}

			}
			else{ // straight route - see how bus_stops vector is created in LoadFromJson
				if (bus_stops.size() < 2){ throw invalid_argument("RouteReqFill invalid straight route");}

				for (int i = 0 ; i < bus_stops.size() - 1; i++){ 		// cycle for switching stops
					double weight_forward = 0;
					double weight_back = 0;
					int span_count = 0;
					auto it_start = bus_stops.begin() + i;
					auto it_delta = it_start;

					for (int j = 0; j < bus_stops.size() - 1 - i; j++){
						it_delta++;
						string from = *it_start;
						string to = *it_delta;
						string point = *(it_delta - 1);
						span_count++;
						Graph::VertexId from_in = stops_.at(from).id; // even inlet
						Graph::VertexId from_out = stops_.at(from).id + 1; // odd outlet
						Graph::VertexId to_in = stops_.at(to).id ; //  even inlet
						Graph::VertexId to_out = stops_.at(to).id + 1 ;  // odd outlet

						weight_forward += static_cast<double> (stop_dists_.at(point).at(to)) / (velocity_ * 1000 / 60) ;
						weight_back += static_cast<double> (stop_dists_.at(to).at(point)) / (velocity_ * 1000 / 60) ;

						Graph::EdgeId id_forward = route_req_graph_.AddEdge({from_out, to_in, weight_forward});
						Graph::EdgeId id_back = route_req_graph_.AddEdge({to_out, from_in, weight_back});

						instructions_[id_forward] = {"Bus", bus_name,weight_forward, span_count};
						instructions_[id_back] = {"Bus", bus_name,weight_back, span_count};

//						cout << endl << "Adding instruction   " << from << "  to  "	<< to  <<  " and reverse"<< endl;
//						cout << "span count is " << span_count << "  and weights are "
//								<< weight_forward << " and " << weight_back	<< endl << endl;
					}
				}
			}
		}
	}

	double velocity_ = 0;
	int wait_ = 0;
	// stops and buses
	unordered_map<Graph::VertexId, string> stop_by_id_; // each stop has even index and 2 vertexes in graph - inlet and outlet. inlet index == stop index, outlet index == stop index + 1
	unordered_map<string, StopInfo> stops_;
	unordered_map<string, vector<string>> bus_routes_;
	unordered_map<string, bool> is_bus_circle_;
	unordered_map<string,DistList> stop_dists_;
	// requests
	vector<tuple<RTYPE, int, string, string>> requests_;
	// request_datastructs
	mutable unordered_map<string, set<string>> stop_req_database;
	mutable unordered_map<string, tuple<int,int,double, double>> bus_req_database;
	mutable Graph::DirectedWeightedGraph<double> route_req_graph_;
	mutable unordered_map<Graph::EdgeId, RouteInstruction> instructions_;
};

void LoadFromJson(TransportManager* manager, istream& input) {
	Json::Document doc = Json::Load(input);

	manager->SetVelocityWait(doc.GetRoot().AsMap().at("routing_settings").AsMap().at("bus_velocity").AsDouble(),
			doc.GetRoot().AsMap().at("routing_settings").AsMap().at("bus_wait_time").AsDouble());

	int StopCount = 0; // for connection between bus stops and vertexes in graph, for each stop even index is for inlet flow and the subsequent odd is for outlet flow

	for (const Json::Node& node : doc.GetRoot().AsMap().at("base_requests").AsArray()){
		const map<string, Json::Node>& req = node.AsMap();
		string type = req.at("type").AsString();
		if (type == "Stop"){
			string stop_name = req.at("name").AsString();
			double lat = req.at("latitude").AsDouble();
			double lon = req.at("longitude").AsDouble();
			const map<string, Json::Node>& stop_dists_map = req.at("road_distances").AsMap();
			vector<pair<string, double>> stop_dists;
			for (const auto& [key, value] : stop_dists_map){
				stop_dists.push_back({key,value.AsDouble()});

			}
			manager->AddStop(StopCount, stop_name, lat, lon, move(stop_dists));
			StopCount += 2; // id for the next stop
		}
		else if(type == "Bus"){
			string bus_name = req.at("name").AsString();
			bool circle_flag = req.at("is_roundtrip").AsBool();
			const vector<Json::Node>& stops_array = req.at("stops").AsArray();
			vector<string> stops;
			for (const auto& node : stops_array){
				stops.push_back(node.AsString());
			}
			manager->AddBus(bus_name,move(stops), circle_flag);
		}
	}

	const vector<Json::Node>& stat_requests = doc.GetRoot().AsMap().at("stat_requests").AsArray();

	for (const Json::Node& node : stat_requests){
		const map<string, Json::Node>& req = node.AsMap();
		string type = req.at("type"). AsString();
		if (type == "Bus"){
			manager->AddRequest(BUS,static_cast<int>(req.at("id").AsDouble()), req.at("name").AsString(), "");
		}
		else if (type == "Stop"){
			manager->AddRequest(STOP,static_cast<int>(req.at("id").AsDouble()), req.at("name").AsString(), "");
		}
		else if (type == "Route"){
			manager->AddRequest(ROUTE,static_cast<int>(req.at("id").AsDouble()), req.at("from").AsString(), req.at("to").AsString());
		}
		else throw invalid_argument("incorrect type of Request");
	}
}

int main(){
//	ifstream f_in ("input.json");
//	ofstream f_out("output.json");
	TransportManager manager;
	LoadFromJson(&manager, cin);
	manager.CreateReqDB();
	manager.ProcessRequests(cout);
	return 0;
}
