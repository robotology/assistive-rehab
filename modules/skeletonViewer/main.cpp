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
#include <unordered_map>
#include <algorithm>
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
    double max_path;
    double last_update;

    vector<double> color;
    Vector z;

    vector<vtkSmartPointer<vtkSphereSource>>   vtk_sphere;
    vector<vtkSmartPointer<vtkPolyDataMapper>> vtk_sphere_mapper;
    vector<vtkSmartPointer<vtkActor>>          vtk_sphere_actor;

    unordered_map<const KeyPoint*,unsigned int> k2id_sphere;

    vector<vtkSmartPointer<vtkQuadric>>        vtk_quadric;
    vector<vtkSmartPointer<vtkSampleFunction>> vtk_quadric_sample;
    vector<vtkSmartPointer<vtkContourFilter>>  vtk_quadric_contours;
    vector<vtkSmartPointer<vtkTransform>>      vtk_quadric_transform;
    vector<vtkSmartPointer<vtkPolyDataMapper>> vtk_quadric_mapper;
    vector<vtkSmartPointer<vtkActor>>          vtk_quadric_actor;

    unordered_map<const KeyPoint*,unordered_map<const KeyPoint*,unsigned int>> kk2id_quadric;

    vtkSmartPointer<vtkCaptionActor2D>         vtk_textActor;

    /****************************************************************/
    bool findCaptionPoint(Vector &p) const
    {
        bool ret=false;
        if (skeleton->operator[](KeyPointTag::head)->isUpdated())
        {
            p=skeleton->operator[](KeyPointTag::head)->getPoint();
            ret=true;
        }
        else
        {
            for (unsigned int i=0; i<skeleton->getNumKeyPoints(); i++)
            {
                auto k=skeleton->operator[](i);
                if (k->isUpdated())
                {
                    p=k->getPoint();
                    ret=true;
                }
            }
        }
        return ret;
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
    void generate_limbs(const KeyPoint *k, const double opacity)
    {
        double a=max_path/100.0;

        vtk_sphere.push_back(vtkSmartPointer<vtkSphereSource>::New());
        vtk_sphere.back()->SetCenter(Vector(k->getPoint()).data());
        vtk_sphere.back()->SetRadius(2.0*a);

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
            double az=0.5*norm(k->getPoint()-c->getPoint());
            az=std::max(0.001,az);

            vtk_quadric.push_back(vtkSmartPointer<vtkQuadric>::New());
            vector<double> coeff(10,0.0);
            coeff[0]=1.0/(a*a);
            coeff[1]=1.0/(a*a);
            coeff[2]=1.0/(az*az);
            vtk_quadric.back()->SetCoefficients(coeff.data());

            vtk_quadric_sample.push_back(vtkSmartPointer<vtkSampleFunction>::New());
            vtk_quadric_sample.back()->SetSampleDimensions(20,20,20);
            vtk_quadric_sample.back()->SetImplicitFunction(vtk_quadric.back());
            vtk_quadric_sample.back()->SetModelBounds(-a,a,-a,a,-az,az);

            vtk_quadric_contours.push_back(vtkSmartPointer<vtkContourFilter>::New());
            vtk_quadric_contours.back()->SetInputConnection(vtk_quadric_sample.back()->GetOutputPort());
            vtk_quadric_contours.back()->GenerateValues(1,1.0,1.0);

            vtk_quadric_mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
            vtk_quadric_mapper.back()->SetInputConnection(vtk_quadric_contours.back()->GetOutputPort());
            vtk_quadric_mapper.back()->SetScalarRange(0.0,1.2);

            vtk_quadric_transform.push_back(vtkSmartPointer<vtkTransform>::New());

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform.back()->Translate((k->getPoint()+m).data());
            align(vtk_quadric_transform.back(),z,m);

            vtk_quadric_actor.push_back(vtkSmartPointer<vtkActor>::New());
            vtk_quadric_actor.back()->SetMapper(vtk_quadric_mapper.back());
            vtk_quadric_actor.back()->SetUserTransform(vtk_quadric_transform.back());
            vtk_quadric_actor.back()->GetProperty()->SetOpacity(opacity);
            vtk_quadric_actor.back()->SetVisibility(k->isUpdated()&&c->isUpdated());
            vtk_renderer->AddActor(vtk_quadric_actor.back());

            kk2id_quadric[k][c]=(unsigned int)vtk_quadric_actor.size()-1;
            generate_limbs(c,opacity);
        }
    }

    /****************************************************************/
    void update_limbs(const KeyPoint *k, const double opacity)
    {
        auto id_sphere=k2id_sphere[k];
        vtk_sphere[id_sphere]->SetCenter(Vector(k->getPoint()).data());
        vtk_sphere_actor[id_sphere]->GetProperty()->SetOpacity(opacity);
        vtk_sphere_actor[id_sphere]->SetVisibility(k->isUpdated());

        for (unsigned int i=0; i<k->getNumChild(); i++)
        {
            auto c=k->getChild(i);
            auto id_quadric=kk2id_quadric[k][c];

            vtk_quadric_transform[id_quadric]->Identity();

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform[id_quadric]->Translate((k->getPoint()+m).data());
            align(vtk_quadric_transform[id_quadric],z,m);
            vtk_quadric_actor[id_quadric]->GetProperty()->SetOpacity(opacity);
            vtk_quadric_actor[id_quadric]->SetVisibility(k->isUpdated()&&c->isUpdated());

            update_limbs(c,opacity);
        }
    }

public:
    /****************************************************************/
    VTKSkeleton(const Property &prop,
                vtkSmartPointer<vtkRenderer> &vtk_renderer_) :
                vtk_renderer(vtk_renderer_)
    {
        z.resize(3,0.0); z[2]=1.0;

        skeleton=unique_ptr<Skeleton>(factory(prop));
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

            double opacity=prop.check("opacity",Value(1.0)).asDouble();
            if (skeleton->getNumKeyPoints()>0)
            {
                max_path=skeleton->getMaxPath();
                auto k=skeleton->operator[](0);
                generate_limbs(k,opacity);

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
            double opacity=prop.check("opacity",Value(1.0)).asDouble();
            if (skeleton->getNumKeyPoints()>0)
            {
                update_limbs(skeleton->operator[](0),opacity);

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
vtkSmartPointer<vtkRenderer> vtk_renderer;
unordered_map<string,unique_ptr<VTKSkeleton>> skeletons;
vector<unordered_map<string,unique_ptr<VTKSkeleton>>::iterator> skeletons_gc_iterators;

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

        if (inputs.size()>0)
        {
            for (auto &input:inputs)
            {
                for (int i=0; i<input.size(); i++)
                {
                    if (Bottle *b=input.get(i).asList())
                    {
                        if (b->check("tag"))
                        {
                            Property prop(b->toString().c_str());
                            string tag=prop.find("tag").asString();
                            if (!tag.empty())
                            {
                                auto s=skeletons.find(tag);
                                if (s==skeletons.end())
                                    skeletons[tag]=unique_ptr<VTKSkeleton>(new VTKSkeleton(prop,vtk_renderer));
                                else
                                    s->second->update(prop);
                            }
                        }
                    }
                }
            }

            inputs.clear();
        }

        if (skeletons_gc_iterators.size()>0)
        {
            for (auto &s:skeletons_gc_iterators)
                skeletons.erase(s);
            skeletons_gc_iterators.clear();
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

    class SkeletonsGC : public RateThread {
        Viewer *viewer;
        void run() override {
            viewer->gc();
        }
    public:
        SkeletonsGC(Viewer *viewer_) : RateThread(1000), viewer(viewer_) { }
    } skeletonsGC;

    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        int x=rf.check("x",Value(0)).asInt();
        int y=rf.check("y",Value(0)).asInt();
        int w=rf.check("w",Value(600)).asInt();
        int h=rf.check("h",Value(600)).asInt();
        double gc_period=rf.check("gc-period",Value(1.0)).asDouble();

        inputPort.open("/skeletonViewer:i");
        skeletonsGC.setRate((int)(gc_period*1000.0));
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

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(0.0,0.0,-2.0);
        vtk_camera->SetFocalPoint(0.0,0.0,0.0);
        vtk_camera->SetViewUp(0.0,-1.0,0.0);
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
        double deadline=0.001*skeletonsGC.getRate();

        for (auto s=begin(skeletons); s!=end(skeletons); s++)
            if (t-s->second->get_last_update()>deadline)
                skeletons_gc_iterators.push_back(s);
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

