#include <windows.h>
#include <iostream>
#include <cmath>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/FirstPersonManipulator>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osg/Geode>
#include <windows.h>
#include <chrono>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include "Cubemap.h"



enum Keys {
    KEY_A = 0x41,
    KEY_B,
    KEY_C,
    KEY_D,
    KEY_E,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_I,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_M,
    KEY_N,
    KEY_O,
    KEY_P,
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_T,
    KEY_U,
    KEY_V,
    KEY_W,
    KEY_X,
    KEY_Y,
    KEY_Z
};

class Camera {
    osg::ref_ptr<osg::Camera> cam;
    osg::Matrixd view;
    osg::Matrixd projection;
    osg::Vec3d eye, center, up;

    void processKeyboardInput(double deltaTime) {
        float speed = 100.0f;
        bool flag = false;
        if (GetAsyncKeyState(KEY_W)) {
            eye +=center * speed * deltaTime;
            flag = true;
        }
        if (GetAsyncKeyState(KEY_A)) {
            osg::Vec3d crossP = center ^ up;
            crossP.normalize();
            eye -= crossP * speed * deltaTime;
            flag = true;
        }
        if (GetAsyncKeyState(KEY_S)) {
            eye -= center * speed * deltaTime;
            flag = true;
        }
        if (GetAsyncKeyState(KEY_D)) {
            osg::Vec3d crossP = center ^ up;
            crossP.normalize();
            eye += crossP * speed * deltaTime;
            flag = true;
        }
        if (flag) {
            view.makeLookAt(eye, eye + center, up);
            cam->setViewMatrix(view);
        }
    }
   
    double yaw = 0, pitch = 0;     
    double xpos = 0, ypos = 0, lastx = 0, lasty = 0;  
    bool firstMouse = true;
   
    void processMouseInput() {
        if (GetAsyncKeyState(KEY_F)){
            POINT p ;
            GetCursorPos(&p);

            if (firstMouse){
                lastx = p.x;
                lasty = p.y;
                firstMouse = false;
            }
            
            xpos = p.x;
            ypos = p.y;
            
            double xoffset = xpos - lastx;
            double yoffset = lasty - ypos;
           
            lastx = xpos;
            lasty = ypos;


            yaw += xoffset;
            pitch += yoffset;

            if (pitch > 89.0f)
                pitch = 89.0f;
            if (pitch < -89.0f)
                pitch = -89.0f;
                     
            center.x() = -std::cos(degToRad(yaw)) * std::cos(degToRad(pitch));
            center.y() = std::sin(degToRad(yaw)) * std::cos(degToRad(pitch));
            center.z() = std::sin(degToRad(pitch));
            center.normalize();
         
            view.makeLookAt(eye,eye + center,up);

            cam->setViewMatrix(view);
            
        }
        else {
            firstMouse = true;
        }
        
    }

    public:
    Camera(osg::ref_ptr<osg::Camera> camera) {
        cam = camera;
        
        projection.makePerspective(80.f, 16 / 9, 1, 1000);

        eye.set(osg::Vec3(300, 300, 300));

        center.x() = -std::cos(degToRad(yaw)) * std::cos(degToRad(pitch));
        center.y() = std::sin(degToRad(yaw)) * std::cos(degToRad(pitch));
        center.z() = std::sin(degToRad(pitch));
        center.normalize();

        up.set(osg::Vec3(0, 0, 1));

        view.makeLookAt(eye,eye + center,up);

        cam->setProjectionMatrix(projection);
        cam->setViewMatrix(view);
    }

    void update(double deltaTime) {
        processKeyboardInput(deltaTime);
        processMouseInput();
    }
    
    osg::ref_ptr<osg::Camera> GetCameraNode() {
        return cam;
    }

};

class Object {
protected:
    osg::ref_ptr<osg::Node> node;//Модуль графической модели
    osg::ref_ptr<osg::MatrixTransform> transform;//Модуль хранящий матрицу трансформаций
    osg::Matrixd tMatrix, // матрица переноса
                 rMatrix, // матрица вращения
                 sMatrix; // матрица масштабирования
    
public:
    void Translate(double x, double y, double z) {//Перенос объекта
        tMatrix = osg::Matrix::translate(x, y, z); //Выставить матрицу переноса на основе параметров метода
        UpdateMatrixTransform();   
 }

    void Rotate(double x, double y, double z) {//Вращение объекта
        rMatrix = osg::Matrix::rotate(x, osg::Vec3d(1, 0, 0), y, osg::Vec3d(0, 1, 0), z, osg::Vec3d(0, 0, 1));
        UpdateMatrixTransform();
    }

    void Scale(double x, double y, double z) {
        sMatrix = osg::Matrix::scale(osg::Vec3d(x,y,z));
        UpdateMatrixTransform();
    }

    virtual void Update(double deltaTime){
    
    };

    void UpdateMatrixTransform() {
        transform->setMatrix(sMatrix * rMatrix * tMatrix);
    }

    osg::ref_ptr<osg::MatrixTransform> GetTransformNode() { //Получить модуль хранящий матрицы перемещения
        return transform;
    }

    Object(osg::ref_ptr<osg::Shape> shape, osg::Vec4d color, osg::ref_ptr<osg::Group> mainGroup) { //Задать графическую модель с помощью osg::ShapeDrawable
        tMatrix = osg::Matrix::translate(0,0,0);
        rMatrix = osg::Matrix::rotate(0, osg::Vec3d(0, 1, 0));
        sMatrix = osg::Matrix::scale(osg::Vec3d(1,1,1));


        osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable;
        shapeDrawable->setShape(shape);//Задать фигуру переданную через параметр конструктора
        shapeDrawable->setColor(color);

        node = shapeDrawable;

        transform = new osg::MatrixTransform;//Создание объекта для хранения матрицы трансформаций
        transform->addChild(node);//Привязать дочерний модуль хранящий графический обьект
        mainGroup->addChild(transform);
    }

    Object(std::string path, osg::ref_ptr<osg::Group> mainGroup) { //Задать графическую модель считав .obj по пути path

        node = osgDB::readRefNodeFile(path);//Загрузить графическую модель из файла

        transform = new osg::MatrixTransform;//Создание объекта для хранения матрицы трансформаций
        transform->addChild(node);//Привязать дочерний модуль хранящий графический обьект
        mainGroup->addChild(transform);
    }
};

class Box : public Object {
    public:
    Box(osg::ref_ptr<osg::Shape> shape, osg::Vec4d color, osg::ref_ptr<osg::Group> mainGroup) 
    : Object(shape,color,mainGroup){}; //Вызов родительского конструктора
        
    float speed = 700.f;
    double step = 0;
    void Update(double deltaTime) override {
        Rotate(0,0,degToRad(step));
        step+=deltaTime*speed;
    }

};


int main(int argc, char** argv) {
    //osg::ArgumentParser arguments(&argc, argv);
    //std::string fileName; //Путь к модели из командной строки
    //arguments.read("--model", fileName);

    osg::ref_ptr<osg::Node> cubemap = createCubemap();
    

    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

    Camera cam(viewer->getCamera());

    osg::ref_ptr<osg::Group> group = new osg::Group;
    

    std::vector<Object*> objects; 

    Object obj("ExampleData/lz.osgt", group),
           obj2("ExampleData/cessnafire.osgt", group),
           obj4(new osg::Cone(osg::Vec3(0,0,0),50,50),osg::Vec4(0,1,0,0), group),
           obj5(new osg::Sphere(osg::Vec3(0, 0, 0), 50), osg::Vec4(0, 0, 1, 0), group);

    Box    obj3(new osg::Box(osg::Vec3(0,0,0),30),osg::Vec4(1,0,0,0),group);
    group->addChild(cubemap);
    
    obj.Scale(5,5,5);
    obj.Translate(0,0,-500);

    obj2.Translate(0,0,300);
    obj3.Translate(0,0,200);    
    obj3.Scale(2,1,1);
    obj3.Rotate(0, 0, 0);
    obj4.Translate(0,100,200);
    obj5.Translate(0, -100, 200);

    objects.push_back(&obj);
    objects.push_back(&obj2);
    objects.push_back(&obj3);
    objects.push_back(&obj4);
    objects.push_back(&obj5);

    viewer->setSceneData(group);
    viewer->setUpViewInWindow(100, 100, 1024, 768);//Запуск в окне
    
    double deltaTime = 0;
    
    while (!viewer->done()){

        auto Time1 = std::chrono::high_resolution_clock::now();

        
        for (size_t i = 0; i < objects.size(); i++){
            objects[i]->Update(deltaTime);
        }

        viewer->frame();
        cam.update(deltaTime);

        auto Time2 = std::chrono::high_resolution_clock::now();
        deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(Time2 - Time1).count();
    }

    return 0;
}