#include "LibsvmWrapper.h"
#include "svm.h"
#include <iostream>

using namespace std;

void LibsvmWrapper::loadModel(char* filename)
{
    this->svmModel= svm_load_model(filename);
}

double LibsvmWrapper::predict(vector<double>& features)
{
    svm_node* svm_data= new svm_node[features.size()+1];
    for (int i= 0; i<features.size(); i++)
    {
            svm_data[i].index= i+1;
            svm_data[i].value= features[i];
    }
    svm_data[features.size()].index= -1;
    svm_data[features.size()].value= -1;

    //for (int i= 0; i<=features.size(); i++)
    //	cerr << svm_data[i].index << ":" << svm_data[i].value << " | " ;
    //cerr << endl;

    double prediction= svm_predict(svmModel, svm_data);


    delete svm_data;

    return prediction;
}

double LibsvmWrapper::predict_probability(std::vector<double>& features, double* probability_estimate)
{
    svm_node* svm_data= new svm_node[features.size()+1];
    for (int i= 0; i<features.size(); i++)
    {
            svm_data[i].index= i+1;
            svm_data[i].value= features[i];
    }
    svm_data[features.size()].index= -1;
    svm_data[features.size()].value= -1;

    //for (int i= 0; i<=features.size(); i++)
    //	cerr << svm_data[i].index << ":" << svm_data[i].value << " | " ;
    //cerr << endl;

    double prediction = svm_predict_probability(svmModel, svm_data, probability_estimate);

    delete svm_data;

    return prediction;
}

int LibsvmWrapper::get_nr_class()
{
    return svm_get_nr_class(svmModel);
}

void LibsvmWrapper::get_labels(int *labels) {
    svm_get_labels(svmModel,labels);
}