#include "detector.h"
#include "src/lineDetector_IDL.h"

using namespace yarp::os;
using namespace yarp::math;

/****************************************************************/
bool Detector::detect(const std::string &line, const int timeout)
{
    std::unique_lock<std::mutex> lck(m_mtx_line_detected);
    if (opcCheck(line)>=0)
    {
        yWarning()<<"Line already inside opc";
        return false;
    }
    this->m_line=line;
    if(m_line2idx.count(line)==0)
    {
        yWarning()<<"This line does not exist";
        return false;
    }
    m_start_detection=true;
    m_line_cnt=0;
    yInfo()<<"Detecting"<<line;
    if (timeout>0)
    {
        m_line_detected.wait_for(lck,std::chrono::seconds(timeout));
    }
    else
    {
        m_line_detected.wait(lck);
    }

    if (line=="start-line" && !m_updated_odom)
    {
        yarp::sig::Matrix wf=SE3inv(m_camFrame* m_lineFrame);
        yarp::sig::Vector tr=wf.getCol(3);
        yarp::sig::Vector rot=(180/M_PI)*dcm2axis(wf.submatrix(0,2,0,2));
        update_odometry(tr[0],tr[1],rot[3]);
    }
    bool ret=create_line_viewer(line);
    create_line_tf(line);

    if (line=="finish-line" && ret)
    {
        m_updated_line_viewer=true;
    }
    m_start_detection=false;

    return ret;
}

/****************************************************************/
bool Detector::reset()
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
//   tvec.clear();
//   rvec.clear();
    for(int i=0;i< m_nlines;i++)
    {
        m_lines_pose_world[i]=yarp::sig::Vector(7,0.0);
        m_lines_filter[i]->init(yarp::sig::Vector(7,0.0));
        m_updated_odom=false;
        std::string l="";
        for (auto &j: m_line2idx)
        {
            if(j.second==i)
            {
                l=j.first;
            }
        }
        delete_line_viewer(l);
        delete_line_tf(l);
        int opc_id=opcCheck(l);
        if (opc_id>=0)
        {
            opcDel(opc_id);
        }

    }
    m_line_detected.notify_all();
    return true;
}

/****************************************************************/
yarp::sig::Matrix Detector::toYarpMat(const cv::Mat &Rmat)
{
    yarp::sig::Matrix R(Rmat.rows,Rmat.cols);
    for(int i=0;i<Rmat.rows;i++)
    {
        for(int j=0;j<Rmat.cols;j++)
        {
            R(i,j)=Rmat.at<double>(i,j);
        }
    }
    return R;
}

/****************************************************************/
yarp::sig::Matrix Detector::update_world_frame(const std::string &line, const yarp::sig::Matrix &R,
                                        const yarp::sig::Vector &pose)
{
    yarp::sig::Matrix M=eye(4);
    if (line=="start-line")
    {
        m_lineFrame.resize(4,4);
        m_lineFrame.setSubmatrix(R,0,0);
        m_lineFrame.setCol(3,pose);
        m_lineFrame.setRow(3,yarp::sig::Vector(3,0.0));
        yarp::sig::Matrix T=SE3inv(m_camFrame* m_lineFrame);
        M=T* m_camFrame;
    }
    if (line=="finish-line")
    {
        M= m_navFrame* m_camFrame;
    }
    return M;
}

/****************************************************************/
void Detector::estimate_line(const yarp::sig::Matrix &wf, const yarp::sig::Vector &pose,
                    const int line_idx)
{
    yarp::sig::Vector rot=yarp::math::dcm2axis(wf.submatrix(0,2,0,2));
    yarp::sig::Vector est_pose_world(7,0.0);
    est_pose_world.setSubvector(0,wf*pose);
    est_pose_world.setSubvector(3,rot);
    m_lines_pose_world[line_idx]= m_lines_filter[line_idx]->filt(est_pose_world);
    yarp::sig::Vector v= m_lines_pose_world[line_idx].subVector(3,5);
    if (norm(v)>0.0)
        {v/=norm(v);}
    m_lines_pose_world[line_idx].setSubvector(3,v);
}

/****************************************************************/
bool Detector::detect_helper(const cv::Mat &inImgMat)
{
    //detect markers
    int line_idx= m_line2idx.at(m_line);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(inImgMat, m_dictionary[line_idx],corners,ids);
    if (ids.size()>0)
    {
        std::vector<std::vector<cv::Point2f> > rejected;
        cv::aruco::GridBoard* curr_board= m_board[line_idx];

        //perform refinement
        cv::aruco::refineDetectedMarkers(inImgMat,curr_board,corners,ids,rejected, m_cam_intrinsic, m_cam_distortion);
        cv::aruco::drawDetectedMarkers(inImgMat,corners,ids);

        //estimate pose
        //use cv::Mat and not cv::Vec3d to avoid issues with
        //cv::aruco::estimatePoseBoard() initial guess
        int valid=cv::aruco::estimatePoseBoard(corners,ids,curr_board, m_cam_intrinsic, m_cam_distortion, m_rvec_v[line_idx], m_tvec_v[line_idx], m_use_initial_guess);
            
        //if at least one board marker detected
        if (valid>0)
        {
            yInfo()<<"n. of valid markers:"<<valid;
            cv::drawFrameAxes(inImgMat, m_cam_intrinsic, m_cam_distortion, m_rvec_v[line_idx], m_tvec_v[line_idx],0.5);
            cv::Mat Rmat;
            cv::Rodrigues(m_rvec_v[line_idx],Rmat);
            yarp::sig::Matrix R=toYarpMat(Rmat);
            yarp::sig::Vector pose({ m_tvec_v[line_idx][0],m_tvec_v[line_idx][1],m_tvec_v[line_idx][2],1.0});
            m_worldFrame=update_world_frame(m_line,R,pose);
            estimate_line(m_worldFrame,pose,line_idx);
            m_line_cnt++;
            if (m_line_cnt> m_line_filter_order)
            {
                int opc_id=opcCheck(m_line);
                opcAdd(m_lines_pose_world[line_idx], m_lines_size[line_idx],opc_id, m_line);
                m_line_detected.notify_all();
                return true;
            }
        }
        else
        {
            yInfo()<<"No board marker detected";
        }
    }
    else
    {
        yInfo()<<"Line not in FOV";
    }
    return false;
}

bool Detector::set_initial_guess(bool flag)
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
    m_use_initial_guess = flag;

    return true;
}

bool Detector::get_initial_guess()
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
    return m_use_initial_guess;
}


/****************************************************************/
yarp::os::Property Detector::get_line(const std::string &line)
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
    if(m_line2idx.count(line)>0)
    {
        int id=opcCheck(line);
        if (id>=0)
        {
            return opcGetLine(id,line);
        }
        else
        {
            return {};
        }
    }
    else
    {
        yWarning()<<"This line does not exist";
        return {};
    }
}
