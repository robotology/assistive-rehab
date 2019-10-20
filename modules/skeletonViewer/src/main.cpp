/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <mutex>
#include <memory>
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iterator>
#include <functional>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkLineSource.h>
#include <vtkQuadric.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkActor.h>
#include <vtkTextActor.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

#include "AssistiveRehab/skeleton.h"
#include "src/skeletonViewer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

/****************************************************************/
class VTKSkeleton
{
protected:
    vtkSmartPointer<vtkRenderer> &vtk_renderer;
    unique_ptr<Skeleton> skeleton;
    double c_length;
    double last_update;

    vector<double> color;
    double opacity;
    Vector z;

    vector<vtkSmartPointer<vtkSphereSource>>   vtk_sphere;
    vector<vtkSmartPointer<vtkPolyDataMapper>> vtk_sphere_mapper;
    vector<vtkSmartPointer<vtkActor>>          vtk_sphere_actor;
    vector<vtkSmartPointer<vtkQuadric>>        vtk_quadric;
    vector<vtkSmartPointer<vtkSampleFunction>> vtk_quadric_sample;
    vector<vtkSmartPointer<vtkContourFilter>>  vtk_quadric_contours;
    vector<vtkSmartPointer<vtkTransform>>      vtk_quadric_transform;
    vector<vtkSmartPointer<vtkPolyDataMapper>> vtk_quadric_mapper;
    vector<vtkSmartPointer<vtkActor>>          vtk_quadric_actor;
    vtkSmartPointer<vtkCaptionActor2D>         vtk_text_actor;

    unordered_map<const KeyPoint*,unsigned int> k2id_sphere;
    unordered_map<const KeyPoint*,unordered_map<const KeyPoint*,unsigned int>> kk2id_quadric;

    /****************************************************************/
    bool update_color(const Property &p)
    {
        if (Bottle *b=p.find("color").asList())
        {
            if (b->size()>=3)
            {
                color=vector<double>{b->get(0).asDouble(),
                                     b->get(1).asDouble(),
                                     b->get(2).asDouble()};
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool findCaptionPoint(Vector &p) const
    {
        if ((*skeleton)[KeyPointTag::head]->isUpdated())
        {
            p=(*skeleton)[KeyPointTag::head]->getPoint();
            return true;
        }
        else
        {
            for (unsigned int i=0; i<skeleton->getNumKeyPoints(); i++)
            {
                auto k=(*skeleton)[i];
                if (k->isUpdated())
                {
                    p=k->getPoint();
                    return true;
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool align(vtkSmartPointer<vtkTransform> &vtk_transform,
               const Vector &v1, const Vector &v2)
    {
        double n1=norm(v1);
        double n2=norm(v2);
        if ((n1>0.0) && (n2>0.0))
        {
            Vector v1_=(1.0/n1)*v1;
            Vector v2_=(1.0/n2)*v2;
            double angle=(180.0/M_PI)*acos(dot(v1_,v2_));
            Vector axis=cross(v1_,v2_);
            vtk_transform->RotateWXYZ(angle,axis.data());
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    void generate_limbs(const KeyPoint *k)
    {
        vtk_sphere.push_back(vtkSmartPointer<vtkSphereSource>::New());
        vtk_sphere.back()->SetCenter(Vector(k->getPoint()).data());
        vtk_sphere.back()->SetRadius(2.0*c_length);

        vtk_sphere_mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
        vtk_sphere_mapper.back()->SetInputConnection(vtk_sphere.back()->GetOutputPort());

        vtk_sphere_actor.push_back(vtkSmartPointer<vtkActor>::New());
        vtk_sphere_actor.back()->SetMapper(vtk_sphere_mapper.back());
        vtk_sphere_actor.back()->GetProperty()->SetColor(color.data());
        vtk_sphere_actor.back()->GetProperty()->SetOpacity(opacity);
        vtk_sphere_actor.back()->SetVisibility(k->isUpdated());
        vtk_renderer->AddActor(vtk_sphere_actor.back());

        k2id_sphere[k]=(unsigned int)vtk_sphere_actor.size()-1;

        for (unsigned int i=0; i<k->getNumChild(); i++)
        {
            auto c=k->getChild(i);
            double cz_length=0.5*norm(k->getPoint()-c->getPoint());
            cz_length=std::max(0.001,cz_length);

            vtk_quadric.push_back(vtkSmartPointer<vtkQuadric>::New());
            vector<double> coeff(10,0.0);
            coeff[0]=1.0/(c_length*c_length);
            coeff[1]=1.0/(c_length*c_length);
            coeff[2]=1.0/(cz_length*cz_length);
            vtk_quadric.back()->SetCoefficients(coeff.data());

            vtk_quadric_sample.push_back(vtkSmartPointer<vtkSampleFunction>::New());
            vtk_quadric_sample.back()->SetSampleDimensions(20,20,20);
            vtk_quadric_sample.back()->SetImplicitFunction(vtk_quadric.back());
            vtk_quadric_sample.back()->SetModelBounds(-c_length,c_length,-c_length,c_length,-cz_length,cz_length);

            vtk_quadric_contours.push_back(vtkSmartPointer<vtkContourFilter>::New());
            vtk_quadric_contours.back()->SetInputConnection(vtk_quadric_sample.back()->GetOutputPort());
            vtk_quadric_contours.back()->GenerateValues(1,1.0,1.0);

            vtk_quadric_mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
            vtk_quadric_mapper.back()->SetInputConnection(vtk_quadric_contours.back()->GetOutputPort());
            vtk_quadric_mapper.back()->ScalarVisibilityOff();

            vtk_quadric_transform.push_back(vtkSmartPointer<vtkTransform>::New());

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform.back()->Translate((k->getPoint()+m).data());
            align(vtk_quadric_transform.back(),z,m);

            vtk_quadric_actor.push_back(vtkSmartPointer<vtkActor>::New());
            vtk_quadric_actor.back()->SetMapper(vtk_quadric_mapper.back());
            vtk_quadric_actor.back()->SetUserTransform(vtk_quadric_transform.back());
            vtk_quadric_actor.back()->GetProperty()->SetColor(color.data());
            vtk_quadric_actor.back()->GetProperty()->SetOpacity(opacity);
            vtk_quadric_actor.back()->SetVisibility(k->isUpdated()&&c->isUpdated());
            vtk_renderer->AddActor(vtk_quadric_actor.back());

            kk2id_quadric[k][c]=(unsigned int)vtk_quadric_actor.size()-1;
            generate_limbs(c);
        }
    }

    /****************************************************************/
    void update_limbs(const KeyPoint *k)
    {
        auto id_sphere=k2id_sphere[k];
        vtk_sphere[id_sphere]->SetCenter(Vector(k->getPoint()).data());
        vtk_sphere[id_sphere]->SetRadius(2.0*c_length);
        vtk_sphere_actor[id_sphere]->GetProperty()->SetColor(color.data());
        vtk_sphere_actor[id_sphere]->GetProperty()->SetOpacity(opacity);
        vtk_sphere_actor[id_sphere]->SetVisibility(k->isUpdated());

        for (unsigned int i=0; i<k->getNumChild(); i++)
        {
            auto c=k->getChild(i);
            auto id_quadric=kk2id_quadric[k][c];

            double cz_length=0.5*norm(k->getPoint()-c->getPoint());
            cz_length=std::max(0.001,cz_length);

            vector<double> coeff(10,0.0);
            coeff[0]=coeff[1]=1.0/(c_length*c_length);
            coeff[2]=1.0/(cz_length*cz_length);
            vtk_quadric[id_quadric]->SetCoefficients(coeff.data());

            vtk_quadric_sample[id_quadric]->SetModelBounds(-c_length,c_length,-c_length,c_length,-cz_length,cz_length);
            vtk_quadric_transform[id_quadric]->Identity();

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform[id_quadric]->Translate((k->getPoint()+m).data());
            align(vtk_quadric_transform[id_quadric],z,m);
            vtk_quadric_actor[id_quadric]->GetProperty()->SetColor(color.data());
            vtk_quadric_actor[id_quadric]->GetProperty()->SetOpacity(opacity);
            vtk_quadric_actor[id_quadric]->SetVisibility(k->isUpdated()&&c->isUpdated());

            update_limbs(c);
        }
    }

public:
    /****************************************************************/
    VTKSkeleton(const Property &prop,
                vtkSmartPointer<vtkRenderer> &vtk_renderer_) :
                vtk_renderer(vtk_renderer_)
    {
        z.resize(3,0.0); z[2]=1.0;
        c_length=0.01;

        skeleton=unique_ptr<Skeleton>(skeleton_factory(prop));
        if (skeleton!=nullptr)
        {
            vector<vector<double>> colors_code;
            colors_code.push_back(vector<double>{213.0/255.0, 47.0/255.0, 65.0/255.0});
            colors_code.push_back(vector<double>{ 58.0/255.0, 79.0/255.0,122.0/255.0});
            colors_code.push_back(vector<double>{ 72.0/255.0,208.0/255.0,154.0/255.0});
            colors_code.push_back(vector<double>{249.0/255.0,196.0/255.0, 71.0/255.0});
            colors_code.push_back(vector<double>{ 96.0/255.0,176.0/255.0,224.0/255.0});
            colors_code.push_back(vector<double>{238.0/255.0,118.0/255.0, 22.0/255.0});

            hash<string> color_hash;
            color=colors_code[color_hash(skeleton->getTag())%colors_code.size()];

            // override color
            update_color(prop);
            opacity=prop.check("opacity",Value(1.0)).asDouble();
            
            if (skeleton->getNumKeyPoints()>0)
            {
                auto k=(*skeleton)[0];
                generate_limbs(k);

                vtk_text_actor=vtkSmartPointer<vtkCaptionActor2D>::New();
                vtk_text_actor->GetTextActor()->SetTextScaleModeToNone();
                vtk_text_actor->SetCaption(skeleton->getTag().c_str());
                vtk_text_actor->BorderOff();
                vtk_text_actor->LeaderOn();
                vtk_text_actor->GetCaptionTextProperty()->SetColor(color.data());
                vtk_text_actor->GetCaptionTextProperty()->SetFontSize(20);
                vtk_text_actor->GetCaptionTextProperty()->FrameOff();
                vtk_text_actor->GetCaptionTextProperty()->ShadowOff();
                vtk_text_actor->GetCaptionTextProperty()->BoldOff();
                vtk_text_actor->GetCaptionTextProperty()->ItalicOff();
                vtk_text_actor->SetVisibility(opacity!=0.0);
                vtk_renderer->AddActor(vtk_text_actor);

                Vector p;
                if (findCaptionPoint(p))
                    vtk_text_actor->SetAttachmentPoint(p.data());
            }
        }

        last_update=Time::now();
    }

    /****************************************************************/
    virtual ~VTKSkeleton()
    {
        for (auto &actor:vtk_sphere_actor)
            vtk_renderer->RemoveActor(actor);
        for (auto &actor:vtk_quadric_actor)
            vtk_renderer->RemoveActor(actor);
        vtk_renderer->RemoveActor(vtk_text_actor);
    }

    /****************************************************************/
    void update(const Property &prop)
    {
        if (skeleton!=nullptr)
        {
            skeleton->update(prop);

            update_color(prop);
            opacity=prop.check("opacity",Value(1.0)).asDouble();
            
            if (skeleton->getNumKeyPoints()>0)
            {
                update_limbs((*skeleton)[0]);

                Vector p;
                if (findCaptionPoint(p))
                    vtk_text_actor->SetAttachmentPoint(p.data());
                vtk_text_actor->GetCaptionTextProperty()->SetColor(color.data());
                vtk_text_actor->SetVisibility(opacity!=0.0);
            }
        }

        last_update=Time::now();
    }

    /****************************************************************/
    double get_last_update() const
    {
        return last_update;
    }
};


/****************************************************************/
class VTKLine
{
protected:
    string name;
    vector<double> line_origin;
    vector<double> line_end;
    vector<double> color;

    vtkSmartPointer<vtkRenderer>       &vtk_renderer;
    vtkSmartPointer<vtkLineSource>      vtk_line;
    vtkSmartPointer<vtkPolyDataMapper>  vtk_line_mapper;
    vtkSmartPointer<vtkActor>           vtk_line_actor;
    vtkSmartPointer<vtkCaptionActor2D>  vtk_text_actor;

public:
    /****************************************************************/
    VTKLine(const string &name_, const vector<double> &line_origin_,
            const vector<double> &line_end_, const vector<double> &color_,
            vtkSmartPointer<vtkRenderer> &vtk_renderer_) :
            name(name_), line_origin(line_origin_), line_end(line_end_),
            color(color_), vtk_renderer(vtk_renderer_) { }

    /****************************************************************/
    void draw()
    {
        vtk_line=vtkSmartPointer<vtkLineSource>::New();
        vtk_line->SetPoint1(line_origin.data());
        vtk_line->SetPoint2(line_end.data());

        vtk_line_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_line_mapper->SetInputConnection(vtk_line->GetOutputPort());

        vtk_line_actor=vtkSmartPointer<vtkActor>::New();
        vtk_line_actor->SetMapper(vtk_line_mapper);
        vtk_line_actor->GetProperty()->SetLineWidth(4);
        vtk_line_actor->GetProperty()->SetColor(color.data());

        vtk_text_actor=vtkSmartPointer<vtkCaptionActor2D>::New();
        vtk_text_actor->GetTextActor()->SetTextScaleModeToNone();
        vtk_text_actor->SetCaption(name.c_str());
        vtk_text_actor->BorderOff();
        vtk_text_actor->LeaderOn();
        vtk_text_actor->GetCaptionTextProperty()->SetColor(color.data());
        vtk_text_actor->GetCaptionTextProperty()->SetFontSize(20);
        vtk_text_actor->GetCaptionTextProperty()->FrameOff();
        vtk_text_actor->GetCaptionTextProperty()->ShadowOff();
        vtk_text_actor->GetCaptionTextProperty()->BoldOff();
        vtk_text_actor->GetCaptionTextProperty()->ItalicOff();
        vtk_text_actor->SetAttachmentPoint(line_origin.data());

        vtk_renderer->AddActor(vtk_line_actor);
        vtk_renderer->AddActor(vtk_text_actor);
    }

    /****************************************************************/
    virtual ~VTKLine()
    {
        vtk_renderer->RemoveActor(vtk_line_actor);
        vtk_renderer->RemoveActor(vtk_text_actor);
    }
};


mutex mtx;
vtkSmartPointer<vtkRenderer> vtk_renderer;

Vector camera_position,camera_focalpoint,camera_viewup;
bool rpc_camera_rx;

vector<Bottle> skeletons_rx;
unordered_map<string,unique_ptr<VTKSkeleton>> skeletons;
unordered_set<string> skeletons_gc_tags;

unordered_set<string> lines_rx;
unordered_map<string,unique_ptr<VTKLine>> lines;
unordered_set<string> lines_gc_tags;

/****************************************************************/
class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    /****************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /****************************************************************/
    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }

    /****************************************************************/
    UpdateCommand() : closing(nullptr) { }

    /****************************************************************/
    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }

    /****************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), 
                 void *vtkNotUsed(callData))
    {
        lock_guard<mutex> lg(mtx);
        vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);

        if (closing!=nullptr)
        {
            if (*closing)
            {
                iren->GetRenderWindow()->Finalize();
                iren->TerminateApp();
                return;
            }
        }

        unordered_set<string> skeletons_prevent_gc_tags;
        if (!skeletons_rx.empty())
        {
            for (auto &sk:skeletons_rx)
            {
                for (int i=0; i<sk.size(); i++)
                {
                    if (Bottle *b1=sk.get(i).asList())
                    {
                        if (b1->check("tag"))
                        {
                            Property prop(b1->toString().c_str());
                            string tag=prop.find("tag").asString();
                            if (!tag.empty())
                            {
                                auto s=skeletons.find(tag);
                                if (s==skeletons.end())
                                    skeletons[tag]=unique_ptr<VTKSkeleton>(new VTKSkeleton(prop,vtk_renderer));
                                else
                                    s->second->update(prop);
                                skeletons_prevent_gc_tags.insert(tag);
                            }
                        }
                        else if (Bottle *b2=b1->find("remove-tags").asList())
                        {
                            for (int j=0; j<b2->size(); j++)
                            {
                                string tag=b2->get(j).asString();
                                auto s=skeletons.find(tag);
                                if (s!=skeletons.end())
                                    skeletons_gc_tags.insert(tag);
                            }
                        }
                    }
                }
            }

            skeletons_rx.clear();
        }

        if (!lines_rx.empty())
        {
            for (auto &name:lines_rx)
                lines[name]->draw();
            lines_rx.clear();
        }

        if (rpc_camera_rx)
        {
            vtkSmartPointer<vtkCamera> vtk_camera=vtk_renderer->GetActiveCamera();
            vtk_camera->SetPosition(camera_position.data());
            vtk_camera->SetFocalPoint(camera_focalpoint.data());
            vtk_camera->SetViewUp(camera_viewup.data());
            yInfo()<<"new camera options received:"
                   <<"position=("<<camera_position.toString(3,3)<<");"
                   <<"focalpoint=("<<camera_focalpoint.toString(3,3)<<");"
                   <<"viewup=("<<camera_viewup.toString(3,3)<<");";
            rpc_camera_rx=false;
        }

        if (!skeletons_gc_tags.empty())
        {
            unordered_set<string> do_gc_tags;
            set_difference(begin(skeletons_gc_tags),end(skeletons_gc_tags),
                           begin(skeletons_prevent_gc_tags),end(skeletons_prevent_gc_tags),
                           inserter(do_gc_tags,end(do_gc_tags)));
            for (auto &tag:do_gc_tags)
            {
                auto s=skeletons.find(tag);
                if (s!=skeletons.end())
                    skeletons.erase(s);
            }
            skeletons_gc_tags.clear();
        }

        if (!lines_gc_tags.empty())
        {
            for (auto &name:lines_gc_tags)
            {
                auto line=lines.find(name);
                lines.erase(line);
            }
            lines_gc_tags.clear();
        }

        iren->GetRenderWindow()->SetWindowName("Skeleton Viewer");
        iren->Render();
    }
};


/****************************************************************/
class Viewer : public RFModule, public skeletonViewer_IDL
{
    bool closing;

    class InputPort : public BufferedPort<Bottle> {
        Viewer *viewer;
        void onRead(Bottle& input) override {
            viewer->process(input);
        }
    public:
        InputPort(Viewer *viewer_) : viewer(viewer_) {
            useCallback();
        }
    } inputPort;

    class SkeletonsGC : public PeriodicThread {
        Viewer *viewer;
        void run() override {
            viewer->gc();
        }
    public:
        SkeletonsGC(Viewer *viewer_) : PeriodicThread(1.0), viewer(viewer_) { }
    } skeletonsGC;

    RpcServer rpcPort;

    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        rpc_camera_rx=false;

        int x=rf.check("x",Value(0)).asInt();
        int y=rf.check("y",Value(0)).asInt();
        int w=rf.check("w",Value(600)).asInt();
        int h=rf.check("h",Value(600)).asInt();
        double gc_period=rf.check("gc-period",Value(1.0)).asDouble();

        vector<double> bg_color={0.7,0.7,0.7};
        if (rf.check("bg-color"))
        {
            if (const Bottle *ptr=rf.find("bg-color").asList())
            {
                size_t len=std::min(bg_color.size(),ptr->size());
                for (size_t i=0; i<len; i++)
                    bg_color[i]=ptr->get(i).asDouble();
            }
        }

        inputPort.open("/skeletonViewer:i");
        rpcPort.open("/skeletonViewer:rpc");
        attach(rpcPort);

        skeletonsGC.setPeriod(gc_period);
        skeletonsGC.start();

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetPosition(x,y);
        vtk_renderWindow->SetSize(w,h);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);
        vtk_renderer->SetBackground(bg_color.data());

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();
        vtk_axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);

        vtk_axes->SetTotalLength((0.1*ones(3)).data());
        vtk_renderer->AddActor(vtk_axes);

        camera_position=camera_focalpoint=camera_viewup=zeros(3);
        camera_position[2]=-2.0;
        camera_viewup[1]=-1.0;

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(camera_position.data());
        vtk_camera->SetFocalPoint(camera_focalpoint.data());
        vtk_camera->SetViewUp(camera_viewup.data());
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(10);

        vtk_updateCallback=vtkSmartPointer<UpdateCommand>::New();
        vtk_updateCallback->set_closing(closing);
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
        
        return true;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return false;
    }

    /****************************************************************/
    void process(const Bottle &input)
    {
        lock_guard<mutex> lg(mtx);
        skeletons_rx.push_back(input);
    }

    /****************************************************************/
    void gc()
    {
        lock_guard<mutex> lg(mtx);
        double t=Time::now();
        double deadline=skeletonsGC.getPeriod();

        for (auto s=begin(skeletons); s!=end(skeletons); s++)
            if (t-s->second->get_last_update()>deadline)
                skeletons_gc_tags.insert(s->first);
    }

    /****************************************************************/
    bool set_camera_position(const double x, const double y,
                             const double z) override
    {
        lock_guard<mutex> lg(mtx);
        camera_position[0]=x;
        camera_position[1]=y;
        camera_position[2]=z;
        rpc_camera_rx=true;
        return true;
    }

    /****************************************************************/
    bool set_camera_focalpoint(const double x, const double y,
                               const double z) override
    {
        lock_guard<mutex> lg(mtx);
        camera_focalpoint[0]=x;
        camera_focalpoint[1]=y;
        camera_focalpoint[2]=z;
        rpc_camera_rx=true;
        return true;
    }

    /****************************************************************/
    bool set_camera_viewup(const double x, const double y,
                           const double z) override
    {
        lock_guard<mutex> lg(mtx);
        camera_viewup[0]=x;
        camera_viewup[1]=y;
        camera_viewup[2]=z;
        rpc_camera_rx=true;
        return true;
    }

    /****************************************************************/
    bool create_line(const string &name,
                     const double x0, const double y0, const double z0,
                     const double x1, const double y1, const double z1,
                     const double r, const double g, const double b) override
    {
        lock_guard<mutex> lg(mtx);
        if (lines.find(name)==lines.end())
        {
            vector<double> line_origin({x0,y0,z0});
            vector<double> line_end({x1,y1,z1});
            vector<double> color({r,g,b});
            lines[name]=unique_ptr<VTKLine>(new VTKLine(name,line_origin,line_end,
                                                        color,vtk_renderer));
            lines_rx.insert(name);
            return true;
        }
        else
        {
            yError()<<"Line already existing";
            return false;
        }
    }

    /****************************************************************/
    bool delete_line(const string &name) override
    {
        lock_guard<mutex> lg(mtx);
        if (lines.find(name)!=lines.end())
        {
            lines_gc_tags.insert(name);
            return true;
        }
        else
        {
            yError()<<"Line not existing";
            return false;
        }
    }

    /****************************************************************/
    bool interruptModule() override
    {
        closing=true;
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        skeletonsGC.stop();
        rpcPort.close();
        inputPort.close();
        return true;
    }

public:
    /****************************************************************/
    Viewer() : closing(false), inputPort(this), skeletonsGC(this) { }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Viewer viewer;
    return viewer.runModule(rf);
}

