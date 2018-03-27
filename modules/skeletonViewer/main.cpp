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
#include <vector>
#include <unordered_map>
#include <algorithm>
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
#include <vtkVectorText.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
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
    double last_update;

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

    vtkSmartPointer<vtkVectorText>             vtk_textSource;
    vtkSmartPointer<vtkPolyDataMapper>         vtk_textMapper;
    vtkSmartPointer<vtkTransform>              vtk_textTransform;
    vtkSmartPointer<vtkActor>                  vtk_textActor;

    /****************************************************************/
    void set_color(vtkSmartPointer<vtkActor> &vtk_actor_id, const bool updated)
    {
        vtk_actor_id->GetProperty()->SetOpacity(updated?1.0:0.5);
        vtk_actor_id->GetProperty()->SetColor(0.1,0.1,updated?0.2:0.1);
    }

    /****************************************************************/
    void generate_limbs(const KeyPoint *k)
    {
        Vector z(3,0.0); z[2]=1.0;
        double ax=1.0/400.0;
        double ay=1.0/400.0;

        vtk_sphere.push_back(vtkSmartPointer<vtkSphereSource>::New());
        vtk_sphere.back()->SetCenter(Vector(k->getPoint()).data());
        vtk_sphere.back()->SetRadius(2.0*ax);

        vtk_sphere_mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
        vtk_sphere_mapper.back()->SetInputConnection(vtk_sphere.back()->GetOutputPort());

        vtk_sphere_actor.push_back(vtkSmartPointer<vtkActor>::New());
        vtk_sphere_actor.back()->SetMapper(vtk_sphere_mapper.back());
        set_color(vtk_sphere_actor.back(),k->isUpdated());
        vtk_renderer->AddActor(vtk_sphere_actor.back());

        k2id_sphere[k]=(unsigned int)vtk_sphere_actor.size()-1;

        for (unsigned int i=0; i<k->getNumChild(); i++)
        {
            auto c=k->getChild(i);
            double az=0.5*norm(k->getPoint()-c->getPoint());
            az=std::max(0.001,az);

            vtk_quadric.push_back(vtkSmartPointer<vtkQuadric>::New());
            vector<double> a(10,0.0);
            a[0]=1.0/(ax*ax);
            a[1]=1.0/(ay*ay);
            a[2]=1.0/(az*az);
            vtk_quadric.back()->SetCoefficients(a.data());

            vtk_quadric_sample.push_back(vtkSmartPointer<vtkSampleFunction>::New());
            vtk_quadric_sample.back()->SetSampleDimensions(20,20,20);
            vtk_quadric_sample.back()->SetImplicitFunction(vtk_quadric.back());
            vtk_quadric_sample.back()->SetModelBounds(-ax,ax,-ay,ay,-az,az);

            vtk_quadric_contours.push_back(vtkSmartPointer<vtkContourFilter>::New());
            vtk_quadric_contours.back()->SetInputConnection(vtk_quadric_sample.back()->GetOutputPort());
            vtk_quadric_contours.back()->GenerateValues(1.0,1.0,1.0);

            vtk_quadric_mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
            vtk_quadric_mapper.back()->SetInputConnection(vtk_quadric_contours.back()->GetOutputPort());
            vtk_quadric_mapper.back()->SetScalarRange(0.0,1.2);

            vtk_quadric_transform.push_back(vtkSmartPointer<vtkTransform>::New());

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform.back()->Translate((k->getPoint()+m).data());
            double n=norm(m);
            if (n>0.0)
            {
                m/=n;
                double angle=(180.0/M_PI)*acos(dot(z,m));
                Vector axis=cross(z,m);
                vtk_quadric_transform.back()->RotateWXYZ(angle,axis.data());
            }

            vtk_quadric_actor.push_back(vtkSmartPointer<vtkActor>::New());
            vtk_quadric_actor.back()->SetMapper(vtk_quadric_mapper.back());
            vtk_quadric_actor.back()->SetUserTransform(vtk_quadric_transform.back());
            set_color(vtk_quadric_actor.back(),k->isUpdated()&&c->isUpdated());
            vtk_renderer->AddActor(vtk_quadric_actor.back());

            kk2id_quadric[k][c]=(unsigned int)vtk_quadric_actor.size()-1;
            generate_limbs(c);
        }
    }

    /****************************************************************/
    void update_limbs(const KeyPoint *k)
    {
        Vector z(3,0.0); z[2]=1.0;

        auto id_sphere=k2id_sphere[k];
        vtk_sphere[id_sphere]->SetCenter(Vector(k->getPoint()).data());
        set_color(vtk_sphere_actor[id_sphere],k->isUpdated());

        for (unsigned int i=0; i<k->getNumChild(); i++)
        {
            auto c=k->getChild(i);
            auto id_quadric=kk2id_quadric[k][c];

            vtk_quadric_transform[id_quadric]->Identity();

            Vector m=0.5*(c->getPoint()-k->getPoint());
            vtk_quadric_transform[id_quadric]->Translate((k->getPoint()+m).data());
            double n=norm(m);
            if (n>0.0)
            {
                m/=n;
                double angle=(180.0/M_PI)*acos(dot(z,m));
                Vector axis=cross(z,m);
                vtk_quadric_transform[id_quadric]->RotateWXYZ(angle,axis.data());
            }

            set_color(vtk_quadric_actor[id_quadric],k->isUpdated()&&c->isUpdated());
            update_limbs(c);
        }
    }

public:
    /****************************************************************/
    VTKSkeleton(const Property &prop,
                vtkSmartPointer<vtkRenderer> &vtk_renderer_) :
                vtk_renderer(vtk_renderer_)
    {
        skeleton=unique_ptr<Skeleton>(factory(prop));
        if (skeleton!=nullptr)
        {
            if (skeleton->getNumKeyPoints()>0)
            {
                auto k=skeleton->operator[](0);
                generate_limbs(k);

                if (!skeleton->getTag().empty())
                {
                    Vector shoulder_center=skeleton->operator[](KeyPointTag::shoulder_center)->getPoint();
                    Vector head=skeleton->operator[](KeyPointTag::head)->getPoint();
                    Vector p=shoulder_center+1.1*(head-shoulder_center);

                    vtk_textSource=vtkSmartPointer<vtkVectorText>::New();
                    vtk_textSource->SetText(skeleton->getTag().c_str());
                    vtk_textMapper=vtkSmartPointer<vtkPolyDataMapper>::New();
                    vtk_textMapper->SetInputConnection(vtk_textSource->GetOutputPort());

                    vtk_textTransform=vtkSmartPointer<vtkTransform>::New();
                    vtk_textTransform->Translate(p.data());
                    vtk_textTransform->Scale(0.01,0.01,0.01);

                    vtk_textActor=vtkSmartPointer<vtkActor>::New();
                    vtk_textActor->SetMapper(vtk_textMapper);
                    vtk_textActor->GetProperty()->SetColor(1.0,0.0,0.0);
                    vtk_textActor->SetUserTransform(vtk_textTransform);
                    vtk_renderer->AddActor(vtk_textActor);
                }

                vtkSmartPointer<vtkCamera> vtk_camera=vtk_renderer->GetActiveCamera();
                vtk_camera->SetPosition((k->getPoint()+2.0*skeleton->getCoronal()).data());
                vtk_camera->SetFocalPoint(k->getPoint().data());
                vtk_camera->SetViewUp(skeleton->getTransverse().data());
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
            if (skeleton->getNumKeyPoints()>0)
            {
                update_limbs(skeleton->operator[](0));
                if (!skeleton->getTag().empty())
                {
                    Vector shoulder_center=skeleton->operator[](KeyPointTag::shoulder_center)->getPoint();
                    Vector head=skeleton->operator[](KeyPointTag::head)->getPoint();
                    Vector p=shoulder_center+1.1*(head-shoulder_center);
                    
                    vtk_textTransform->Identity();
                    vtk_textTransform->Translate(p.data());
                    vtk_textTransform->Scale(0.01,0.01,0.01);
                }
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

    class InputPort : public BufferedPort<Property> {
        Viewer *viewer;
        void onRead(Property& input) override {
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

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;     
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        inputPort.open("/skeletonViewer:i");
        skeletonsGC.start();

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();     
        vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(0.0,0.0,1.0);
        vtk_camera->SetFocalPoint(0.0,0.0,0.0);
        vtk_camera->SetViewUp(0.0,1.0,0.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(1);

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
    void process(const Property &input)
    {
        if (input.check("tag"))
        {
            LockGuard lg(mutex);
            string tag=input.find("tag").asString();
            auto s=skeletons.find(tag);
            if (s==skeletons.end())
                skeletons[tag]=unique_ptr<VTKSkeleton>(new VTKSkeleton(input,vtk_renderer));
            else
                s->second->update(input);
        }
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

