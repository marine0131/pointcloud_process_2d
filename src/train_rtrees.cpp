#include <vector>
#include <opencv2/opencv.hpp>

#include "utils.h"


using namespace std;

struct Args{
    string training_data;
    string model_name;
    int max_depth;
    int min_sample_count;
    float regress_accuracy;
    int max_categories;
    bool calc_var_importance;
    int active_var_count;
    Args()
    {
        model_name = "rt_output.yaml";
        max_depth = 8;
        min_sample_count = 20;
        regress_accuracy = 0.f;
        max_categories = 10;
        calc_var_importance = true;
        active_var_count = 5;
    }
};

class PcRtrees
{ 
    private:
        string training_data_;
        string model_name_;
        int max_depth_;
        int min_sample_count_;
        float regress_accuracy_;
        int max_categories_;
        bool calc_var_importance_;
        int active_var_count_;

    public:

        PcRtrees(Args args)
        {
            training_data_ = args.training_data;
            model_name_ = args.model_name;
            max_depth_ = args.max_depth;
            min_sample_count_ = args.min_sample_count;
            regress_accuracy_ = args.regress_accuracy;
            max_categories_ = args.max_categories;
            calc_var_importance_ = args.calc_var_importance;
            active_var_count_ = args.active_var_count;
        }

        void train(cv::Mat* data, cv::Mat* response)
        {
            cout << "training ..." <<endl;
            int nfeatures = data->cols;
            int nsamples = data->rows;
            int ntrain_samples = (int)(nsamples*0.8);
            cout << "samples for training: " << ntrain_samples <<endl;

            cv::Mat sample_idx = cv::Mat::zeros(1, nsamples, CV_8U);
            cv::Mat var_type(nfeatures+1, 1, CV_8U);
            var_type.setTo(cv::Scalar::all(cv::ml::VAR_ORDERED));
            var_type.at<uchar>(nfeatures) = cv::ml::VAR_CATEGORICAL;

            //training data
            cv::Ptr<cv::ml::TrainData> tdata = cv::ml::TrainData::create(
                    *data, cv::ml::ROW_SAMPLE, *response, cv::noArray(),
                    sample_idx, cv::noArray(), var_type);

            cv::Ptr<cv::ml::RTrees> model = cv::ml::RTrees::create();
            // max depth of tree
            model->setMaxDepth(max_depth_);
            // min samples for train a tree
            model->setMinSampleCount(min_sample_count_);
            // criteria of termination
            model->setRegressionAccuracy(regress_accuracy_);
            // if built surrogate splits
            model->setUseSurrogates(false);
            // max cluster number
            model->setMaxCategories(max_categories_);
            // prior prob
            model->setPriors(cv::Mat());
            // if calculate importance of all variance
            model->setCalculateVarImportance(calc_var_importance_);
            // 
            model->setActiveVarCount(active_var_count_);
            // terminate criteria
            float iter_criteria = cv::TermCriteria::MAX_ITER+
                (0.01f>0 ? cv::TermCriteria::EPS : 0);
            model->setTermCriteria(cv::TermCriteria(
                        iter_criteria, 100, 0.01f));
            // train
            model->train(tdata);
            cout << "training done"<<endl;

            // save model
            if(!model_name_.empty())
            {
                model->save(model_name_);
                cout << "saving model to " << model_name_ << endl;
            }

            // accuracy
            float train_hr=0, test_hr=0;
            for(int i = 0; i < nsamples; i++)
            {
                cv::Mat sample = data->row(i);

                //predict
                float r = model->predict(sample);
                // cout << response->at<float>(i) << ": " << r <<endl;
                r = abs(r-response->at<float>(i)) <= 0.1?1.f:0.f;

                //count
                if(i < ntrain_samples)
                    train_hr += r;
                else
                    test_hr += r;
            }

            train_hr /= ntrain_samples;
            test_hr /= nsamples - ntrain_samples;
            cout << "train precision: " << train_hr*100 << "%, test presicion: " << test_hr*100 << "%" << endl;

            // number of trees
            cout << "Number of trees: " << model->getRoots().size() <<endl;

            //importance of variance
            cv::Mat var_importance = model->getVarImportance();
            if(!var_importance.empty())
            {
                double rt_imp_sum = sum(var_importance)[0];
                cout << "var#\timportance(in %): "<<endl;
                int n = (int)var_importance.total();
                for(int i = 0; i< n; ++i)
                    printf("%-2d\t%-4.1f\n", i, 100.f*var_importance.at<float>(i)/rt_imp_sum);
            }
            
        }
};

void vec2mat(vector<vector<float> >* vec, cv::Mat* mat, cv::Mat* response)
{
    int row = vec->size();
    int col = (*vec)[0].size();

    float* pmat = NULL;
    float* pres = NULL;

    for(int i = 0; i < row; ++i)
    {
        pmat = mat->ptr<float>(i);
        pres = response->ptr<float>(i);
        for(int j=0; j < col-1; ++j)
            pmat[j] = (*vec)[i][j];
        pres[0] = (*vec)[i][col-1];
    }
}

  
int main(int argc, char** argv)
{ 
    Args args;
    vector<string> arg_vec(argv+1, argv+argc);
    for(vector<string>::iterator it = arg_vec.begin(); it!= arg_vec.end(); ++it)
    {
        int pos = it->find("=");
        string arg_string = it->substr(2, pos-2);
        if(arg_string.compare("help") == 0){
            cout << "--trainning_data=feat.xml\n"
                << "--model_name=model.yaml\n"
                << "--max_depth=8\n" 
                << "--min_sample_count=20\n"
                << "--regress_accuracy=0.f\n"
                << "--max_categories=10\n"
                << "--calc_var_importanc=true\n"
                << "--active_var_count=5" <<endl;
            return 0;
        }
        else if(arg_string.compare("training_data") == 0){
            args.training_data = it->substr(pos+1);
        }
        else if(arg_string.compare("model_name") == 0){
            args.model_name = it->substr(pos+1);
        }
        else if(arg_string.compare("max_depth") == 0){
            args.max_depth = stoi(it->substr(pos+1));
        }
        else if(arg_string.compare("min_sample_count") == 0){
            args.min_sample_count = stoi(it->substr(pos+1));
        }
        else if(arg_string.compare("regress_accuracy") == 0){
            args.regress_accuracy = stof(it->substr(pos+1));
        }
        else if(arg_string.compare("max_categories") == 0){
            args.max_categories= stoi(it->substr(pos+1));
        }
        else if(arg_string.compare("calc_var_importance") == 0){
            args.calc_var_importance = it->substr(pos+1)=="true"?true:false;
        }
        else if(arg_string.compare("active_var_count") == 0){
            args.active_var_count = stoi(it->substr(pos+1));
        }
        else{
            cout << "error: input the right arguments: " << arg_string<< endl;
            return 0;
        }
    }
    if(args.training_data.empty())
    {
        cout << "no training data specified!" <<endl;
        return -1;
    }


    PcRtrees pctree(args);

    vector<vector<float> > vec;
    read_2dmat_from_file(args.training_data, &vec);

    cv::Mat data(vec.size(), vec[0].size()-1, CV_32F);
    cv::Mat response(vec.size(), 1, CV_32F);
    vec2mat(&vec, &data, &response);

    cout << "get samples: " << data.rows << ", feature size: " << data.cols <<endl;
    pctree.train(&data, &response);
   
}


