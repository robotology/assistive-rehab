#include "iostream"
#include "vector"
#include "matio.h"
//#include "/home/vvasco/dev/matio/src/mat73.c"

#include "yarp/sig/Vector.h"

using namespace std;
using namespace yarp::sig;

//int main()
//{
//    mat_t *matfp;
////    matvar_t *matvar;
//    matvar_t *matvar1;
//    matvar_t *matvar2;

//    //-------------CREATE MATRIX--------------
//    size_t n = 1;
//    size_t m = 4;
////    vector<vector<double>> keypoints(n, vector<double>(m));
//    double keypoints[n][m];


//    //------------MATIO: CREATE .MAT FILE---------------------
//    matfp = Mat_CreateVer("test_data_fdg.mat",NULL,MAT_FT_MAT73);
//    if (matfp == NULL)
//    {
//        cout << "Error creating MAT file";
//        return EXIT_FAILURE;
//    }

//    //------------MATIO: WRITE TO FILE------------------
//    size_t dims[2] = {n,m};
//    for(int k=0; k<3; k++)
//    {
//        // looping through outer vector vec
//        for (int i = 0; i < n; i++) {
//            // looping through inner vector vec[i]
//            for (int j = 0; j < m; j++) {
//                keypoints[i][j] = k*(i*n + j);
//    //            cout << keypoints[i][j] << " ";
//            }
//    //        cout << endl;
//        }

//        matvar1 = Mat_VarCreate("elbow_left",MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&keypoints,0);
//        matvar2 = Mat_VarCreate("elbow_right",MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&keypoints,0);

//        if (matvar1 != NULL && matvar2 != NULL)
//        {
//            Mat_VarWriteAppend(matfp,matvar1,MAT_COMPRESSION_NONE,1);
//            Mat_VarWriteAppend(matfp,matvar2,MAT_COMPRESSION_NONE,1);
//        }
//    }


//    //------------MATIO: FREE VARIABLES------------------
//    Mat_VarFree(matvar1);
//    Mat_VarFree(matvar2);

//    Mat_Close(matfp);

//    return EXIT_SUCCESS;
//}

//int main()
//{
//    mat_t    *matfp;
//    matvar_t *matvar;
//    matvar_t *field;
//    const char *fields[2] = {"elbow_left","elbow_right"};
//    int nfields = 2;
////    double       data1 = 1, data2 = 2;

//    size_t n = 1;
//    size_t m = 3;
//    double data1[n][m], data2[n][m];
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < m; j++) {
//            data1[i][j] = (i*n + j);
//            data2[i][j] = 2.0*(i*n + j);
//            //            cout << keypoints[i][j] << " ";
//        }
//    }

//    //--------- MATIO: CREATE .MAT FILE---------------
//    matfp = Mat_CreateVer("test_data_fdg.mat",NULL,MAT_FT_MAT73);
//    if ( NULL == matfp ) {
//        fprintf(stderr,"Error opening MAT file %s\n","test_data_fdg.mat");
//        return EXIT_FAILURE;
//    }

//    //------------MATIO: CREATE STRUCT---------------
//    size_t dims[2] = {1,1};
////    dims[0] = 1; dims[1] = 1;
//    matvar = Mat_VarCreateStruct("keypoints",2,dims,fields,nfields);
//    if ( NULL == matvar ) {
//        Mat_Close(matfp);
//        return EXIT_FAILURE;
//    }

//    size_t dims_fields[2] = {n,m};
//    field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_fields,&data1,
//                MAT_F_DONT_COPY_DATA);
//    Mat_VarSetStructFieldByName(matvar, "elbow_left", 0, field);

//    field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_fields,&data2,
//                MAT_F_DONT_COPY_DATA);
//    Mat_VarSetStructFieldByName(matvar, "elbow_right", 0, field);


//    hid_t* id;
//    hsize_t* sz;
//    Mat_VarWriteAppendNextType73(1,matvar,"test_data_fdg.mat", id,sz,1);
////    Mat_VarWriteAppend(matfp,matvar,MAT_COMPRESSION_NONE,0);
//    Mat_VarFree(matvar);

//    Mat_Close(matfp);
//    return EXIT_SUCCESS;
//}


void writeStructToMat(string name, const vector< vector< pair<string,Vector> > >& keypoints_skel, mat_t* matfile)
{
    size_t dims[2] = {1,1}; //static_cast<size_t>(keypoints_skel[0].size())};

    int nSamples = keypoints_skel[0].size();

    int numFields = 2;
    const char *fields[numFields] = {"elbow_left","elbow_right"};
    matvar_t *matvar = Mat_VarCreateStruct(name.c_str(),2,dims,fields,numFields);

    vector<double> field_vector;
    field_vector.resize(3*nSamples);

    cout << "START" << endl;
    size_t dims_field[2] = {3,keypoints_skel[0].size()}; //static_cast<size_t>(keypoints_skel.size())};
    for(int i=0; i<keypoints_skel.size(); i++)
    {
        field_vector.clear();
        for(int j=0; j<keypoints_skel[i].size(); j++)
        {
            field_vector[3*j] = keypoints_skel[i][j].second[0];
            field_vector[3*j+1] = keypoints_skel[i][j].second[1];
            field_vector[3*j+2] = keypoints_skel[i][j].second[2];
            cout << keypoints_skel[i][j].first << " " << field_vector[3*j] << " " << field_vector[3*j+1] << endl;
        }
        matvar_t *field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field,field_vector.data(),MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[i], 0, field);
    }

    Mat_VarWrite(matfile,matvar,MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);

}

//void writeKeypointsToFile(vector<double>& times, vector<vector<double>>& keypoints, vector<string>& keypoints_names)
void writeKeypointsToFile(vector<double>& times, vector< vector< pair<string,Vector> >>& keypoints_skel)
 {
     // Use MATIO to write the results in a .mat file
     std::string filename = "test_data_fdg.mat";
     mat_t *matfp = Mat_CreateVer(filename.c_str(),NULL,MAT_FT_MAT73);
     if (matfp == NULL)
         cout << "Error creating MAT file";

     size_t nSamples = times.size(); //m_ts.size();
     size_t dims[2] = {1,nSamples};
     matvar_t *matvar = Mat_VarCreate("Time_samples", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, times.data(), 0);

     Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
     Mat_VarFree(matvar);

     // Save keypoint names
     writeStructToMat("Keypoints", keypoints_skel, matfp);

     Mat_Close(matfp);
     cout << "Keypoints saved to file " << filename.c_str() << endl;
 }



//int main()
//{

//    //-------------CREATE MATRIX--------------
//    size_t nSamples = 10; //times
//    size_t m = 3;
//    int nKeypoints=2;

////    vector<vector<double>> keypoints(n, vector<double>(m));
//    vector<Vector> keypoints;
////    double keypoints[n][m];

//    vector<double> times(nSamples);

//    vector<string> keypoints_names(nKeypoints);
//    keypoints_names[0] = "elbow_left";
//    keypoints_names[1] = "elbow_right";

//    keypoints.resize(nSamples);
//    for(int k=0; k<nKeypoints; k++)
//    {
//        cout << keypoints_names[k] << endl;
//        for (int i = 0; i<nSamples; i++)
//        {
//            times[i] = i;
////            cout << times[i] << " ";
//            keypoints[i].resize(m);
//            // looping through inner vector vec[i]
//            for (int j = 0; j < m; j++)
//            {
//                keypoints[i][j] += (i+j)*(k+1); // (i*nSamples + j);
//                cout << keypoints[i][j] << " ";
//            }
//            cout << endl;
//        }
//    }

//    //-------------CREATE DATA STRUCTURE--------------
//    vector< vector <pair<string,Vector>> > keypoints_skel;
//    keypoints_skel.resize(nKeypoints);
//    for(int i=0; i<nKeypoints; i++)
//    {
//        keypoints_skel[i].resize(nSamples);
//        for(int j=0; j<nSamples; j++)
//        {
//            pair<string,Vector> pairToWrite = make_pair(keypoints_names[i].c_str(), keypoints[i]);
//            keypoints_skel[i][j] = pairToWrite;
//            cout << keypoints_skel[i][j].first << " "
//                 << keypoints_skel[i][j].second[0] << " " << keypoints_skel[i][j].second[1] << " " << keypoints_skel[i][j].second[2] << endl;
//        }

//    }

//    //------------MATIO: WRITE TO FILE------------------
//    writeKeypointsToFile(times, keypoints_skel);


//    return EXIT_SUCCESS;
//}


int main()
{
    mat_t    *matfp;
    matvar_t *matvar;
    matvar_t *field;
    const char *fields[2] = {"field1","field2"};
    double       data1 = 1, data2 = 2;
    size_t       dims[2] = {1, 1};

    matfp = Mat_Open("test_data_fdg.mat",MAT_ACC_RDWR);
    if ( NULL == matfp ) {
        cout << "Error opening MAT file \n";
        return EXIT_FAILURE;
    }

    dims[0] = 1; dims[1] = 1;
    matvar = Mat_VarCreateStruct("a",2,dims,fields,2);
    if ( NULL == matvar ) {
        Mat_Close(matfp);
        return EXIT_FAILURE;
    }

    field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&data1,
                MAT_F_DONT_COPY_DATA);
    Mat_VarSetStructFieldByName(matvar, "field1", 0, field);

    field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&data2,
                MAT_F_DONT_COPY_DATA);
    Mat_VarSetStructFieldByName(matvar, "field2", 0, field);

    Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);

    Mat_Close(matfp);
    return EXIT_SUCCESS;
}

//int main()
//{
//    mat_t *matfp;
//    const char *FILENAME = "myfile.mat";

//    //Open file
//    matfp = Mat_CreateVer(FILENAME, NULL, MAT_FT_MAT5);

//    //string
//    const char* fieldname1 = "MyStringVariable";
//    char* mystring = "Text";
//    size_t dim1[2] = { 1, sizeof(mystring)/sizeof(mystring[0]) };
//    cout << sizeof(mystring) << " " << sizeof(mystring[0]);
//    matvar_t *variable1 = Mat_VarCreate(fieldname1, MAT_C_CHAR, MAT_T_UTF8, 2, dim1, mystring, 0);
//    Mat_VarWrite(matfp, variable1, MAT_COMPRESSION_NONE); //or MAT_COMPRESSION_ZLIB
//    Mat_VarFree(variable1);

//}
