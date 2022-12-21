#include "json.h"
#include <cmath>
#include <iostream>

using namespace std;

namespace Json {

Document::Document(Node root) : root(move(root)) {
}

const Node& Document::GetRoot() const {
	return root;
}

Node LoadNode(istream& input);

Node LoadArray(istream& input) {
	vector<Node> result;

	for (char c; input >> c && c != ']'; ) {
		if (c != ',') {
			input.putback(c);
		}
		result.push_back(LoadNode(input));
	}

	return Node(move(result));
}

Node LoadDoubleBool(istream& input) {
	while (input.peek() == ' ' || input.peek() == '\n'){
		input.ignore(1);
	}

	if (input.peek() == 't'){
		input.ignore(4);
		return Node(true);
	}
	if (input.peek() == 'f'){
		input.ignore(5);
		return Node(false);
	}
	double result = 0;

	bool is_minus = false;

	if (input.peek() == '-'){
		is_minus = true;
		input.ignore(1);
	}

	while(isdigit(input.peek())){
		result *= 10;
		result += input.get() - '0';
	}
	if (input.peek() == '.'){
		input.ignore(1);
	}
	int count = 1;
	while(isdigit(input.peek())){
		result += (input.get() - '0') / pow(10,count);
		count++;
	}
	if (is_minus){
		result *= -1.0;
	}
	//cerr << result;
	return Node(result);
}

Node LoadString(istream& input) {
	string line;
	getline(input, line, '"');
	//cerr << line;
	return Node(move(line));
}

Node LoadDict(istream& input) {
	map<string, Node> result;
	while (input.peek() == ' ' || input.peek() == '\n'){
		input.ignore(1);
	}

	for (char c; input >> c && c != '}'; ) {
		if (c == ',') {
			input >> c;
			//cerr << c;
		}
		while (input.peek() == ' ' || input.peek() == '\n'){
			input.ignore(1);
		}
		string key = LoadString(input).AsString();
		input >> c;
		//cerr << c;
		result.emplace(move(key), LoadNode(input));
	}

	return Node(move(result));
}

Node LoadNode(istream& input) {
	while (input.peek() == ' ' || input.peek() == '\n'){
		input.ignore(1);
	}
	char c;
	input >> c;
	//cerr << c;

	if (c == '[') {
		return LoadArray(input);
	} else if (c == '{') {
		return LoadDict(input);
	} else if (c == '"') {
		return LoadString(input);
	} else {
		input.putback(c);
		return LoadDoubleBool(input);
	}
}

Document Load(istream& input) {
	return Document{LoadNode(input)};
}

}



//Node LoadInt(istream& input) {
//	int result = 0;
//	while (isdigit(input.peek())) {
//		result *= 10;
//		result += input.get() - '0';
//	}
//	return Node(result);
//}

