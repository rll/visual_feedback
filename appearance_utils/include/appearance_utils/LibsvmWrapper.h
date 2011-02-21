#ifndef LIBSVMWRAPPER_H
#define LIBSVMWRAPPER_H

#include <vector>
#include "svm.h"

class LibsvmWrapper
{
    public:
    svm_model* svmModel;

    void loadModel(char* filename);
    double predict(std::vector<double>& features);
    double predict_probability(std::vector<double>& features, double* probability_estimate);
    int get_nr_class();
    void get_labels(int *labels);
};

#endif
