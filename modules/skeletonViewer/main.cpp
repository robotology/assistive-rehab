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
    vtkSmartPointer<vtkCaptionActor2D>         vtk_textActor;

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

                vtk_textActor=vtkSmartPointer<vtkCaptionActor2D>::New();
                vtk_textActor->GetTextActor()->SetTextScaleModeToNone();
                vtk_textActor->SetCaption(skeleton->getTag().c_str());
                vtk_textActor->BorderOff();
                vtk_textActor->LeaderOn();
                vtk_textActor->GetCaptionTextProperty()->SetColor(color.data());
                vtk_textActor->GetCaptionTextProperty()->SetFontSize(20);
                vtk_textActor->GetCaptionTextProperty()->FrameOff();
                vtk_textActor->GetCaptionTextProperty()->ShadowOff();
                vtk_textActor->GetCaptionTextProperty()->BoldOff();
                vtk_textActor->GetCaptionTextProperty()->ItalicOff();
                vtk_renderer->AddActor(vtk_textActor);

                Vector p;
                if (findCaptionPoint(p))
                    vtk_textActor->SetAttachmentPoint(p.data());
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
        vtk_renderer->RemoveActor(vtk_textActor);
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
                    vtk_textActor->SetAttachmentPoint(p.data());
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


Mutex mutex;
vector<Bottle> inputs;
bool rpc_command_rx;
Vector camera_position,camera_focalpoint,camera_viewup;
vtkSmartPointer<vtkRenderer> vtk_renderer;
unordered_map<string,unique_ptr<VTKSkeleton>> skeletons;
unordered_set<string> skeletons_gc_tags;

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
        LockGuard lg(mutex);
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
        if (!inputs.empty())
        {
            for (auto &input:inputs)
            {
                for (int i=0; i<input.size(); i++)
                {
                    if (Bottle *b1=input.get(i).asList())
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

            inputs.clear();
        }

        if (rpc_command_rx)
        {
            vtkSmartPointer<vtkCamera> vtk_camera=vtk_renderer->GetActiveCamera();
            vtk_camera->SetPosition(camera_position.data());
            vtk_camera->SetFocalPoint(camera_focalpoint.data());
            vtk_camera->SetViewUp(camera_viewup.data());
            yInfo()<<"new camera options received:"
                   <<"position=("<<camera_position.toString(3,3)<<");"
                   <<"focalpoint=("<<camera_focalpoint.toString(3,3)<<");"
                   <<"viewup=("<<camera_viewup.toString(3,3)<<");";
            rpc_command_rx=false;
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

        iren->GetRenderWindow()->SetWindowName("Skeleton Viewer");
        iren->Render();
    }
};


/****************************************************************/
class Viewer : public RFModule
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
    bool configure(ResourceFinder &rf) override
    {
        rpc_command_rx=false;

        int x=rf.check("x",Value(0)).asInt();
        int y=rf.check("y",Value(0)).asInt();
        int w=rf.check("w",Value(600)).asInt();
        int h=rf.check("h",Value(600)).asInt();
        double gc_period=rf.check("gc-period",Value(1.0)).asDouble();

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
        vtk_renderer->SetBackground(0.1,0.2,0.2);

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
        LockGuard lg(mutex);
        inputs.push_back(input);
    }

    /****************************************************************/
    void gc()
    {
        LockGuard lg(mutex);
        double t=Time::now();
        double deadline=skeletonsGC.getPeriod();

        for (auto s=begin(skeletons); s!=end(skeletons); s++)
            if (t-s->second->get_last_update()>deadline)
                skeletons_gc_tags.insert(s->first);
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        LockGuard lg(mutex);
        if (command.size()>0)
        {
            string cmd=command.get(0).asString();
            if (command.size()>1)
            {
                Property options;
                command.tail().write(options);
                if (cmd=="set_camera")
                {
                    if (options.check("position"))
                    {
                        options.find("position").asList()->write(camera_position);
                        rpc_command_rx=true;
                    }
                    if (options.check("focalpoint"))
                    {
                        options.find("focalpoint").asList()->write(camera_focalpoint);
                        rpc_command_rx=true;
                    }
                    if (options.check("viewup"))
                    {
                        options.find("viewup").asList()->write(camera_viewup);
                        rpc_command_rx=true;
                    }
                }
            }
        }

        reply.addVocab(Vocab::encode(rpc_command_rx?"ack":"nack"));
        return true;
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

