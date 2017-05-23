//
//  prev_mesh.cpp
//  test_leap_motion
//
//  Created by 陆婧涵 on 26/3/17.
//  Copyright © 2017 陆婧涵. All rights reserved.
//

#include "prev_mesh.hpp"
#include <stdlib.h>
#include <GLUT/glut.h>
#include <stdio.h>
#include <iostream>
#include <strstream>
#include <fstream>
#include <math.h>
#include <string>
#include "Leap.h"
using namespace Leap;
using namespace std;

#define OBJ_WIREFRAME	0
#define OBJ_POINTS		1
#define OBJ_FLAT		2
#define OBJ_SMOOTH		3

#define TRANSFORM_NONE    0
#define TRANSFORM_ROTATE  1
#define TRANSFORM_SCALE 2
#define TRANSFORM_TRANSLATE 3

static int obj_mode = 3;
static int xform_mode = 0;


GLfloat light0_ambient[] = { 0.1f, 0.1f, 0.3f, 1.0f };
GLfloat light0_diffuse[] = { .6f, .6f, 1.0f, 1.0f };
GLfloat light0_position[] = { .5f, .5f, 1.0f, 0.0f };

GLfloat light1_ambient[] = { 0.1f, 0.1f, 0.3f, 1.0f };
GLfloat light1_diffuse[] = { .9f, .6f, 0.0f, 1.0f };
GLfloat light1_position[] = { -1.0f, -1.0f, 1.0f, 0.0f };

 int prev_frame=0;
int vnumb = 0;
int fnumb = 0;

int projection = 0;
int draw_coordinate = 1;
int bounding_box = 0;
int draw_ground = 1;
const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};


float prev_x = 0, prev_y = 0,prev_z=0;
float new_x = 0, new_y = 0,new_z=0;
float x_tran = 0.0;
float y_tran = 0.0;
float z_tran = 0.0;

float prev_x_angle = 0, prev_y_angle = 0,prev_z_angle=0;
float new_x_angle = 0, new_y_angle = 0,new_z_angle=0;
float x_angle = 0.0;
float y_angle = 0.0;
float z_angle=0.0;


float new_scale=1;
float new_scale_1=1,new_scale_2=1;
float prev_scale_1=1,prev_scale_2=1;
float prev_scale=1;
float scale_size = 1;



float xmax, xmin;
float ymax, ymin;
float zmax, zmin;
float scale;

string fname = "bunny.m";
//==================================================================================================================================
struct Vertex3f
{
    float x, y, z;
};
Vertex3f* vertexarr;
Vertex3f center;

//=================================================================================================================================================
struct Face3i
{
    int vertoffa[3];
}; Face3i *facearr;
//==================================================================================================================================
struct HE_vert;
struct HE_face;

struct HE_edge {
    HE_vert* vert;
    HE_edge* pair;
    HE_face* face;
    HE_edge* prev;
    HE_edge* next;
};
HE_edge *Edge;

struct HE_vert {
    float x, y, z;
    HE_edge* edge;
    float vnormalx, vnormaly, vnormalz;
    int num;
};
HE_vert *Vertex;

struct HE_face {
    HE_edge* edge;
    float fnormalx, fnormaly, fnormalz;
};
HE_face* Face;

struct HE_pair {
    int x, y;
};
HE_pair *findpair;

class SampleListener : public Listener {
public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    
private:
};
//==================================================================================================================================

int halfCompute(int b, int a) {
    
    int half;
    int middle;
    int start = 0;
    int end = 3 * fnumb - 1;
    
    while (start <= end)
    {
        middle = start + (end - start) / 2;
        if (b == findpair[middle].x)
        {
            half = middle;
            do
            {
                if (a == findpair[half].y)
                {
                    return half;
                }
                else half++;
            } while (b == findpair[half].x);
            half = middle;
            do {
                if (a == findpair[half].y)
                    return half;
                else half--;
            } while (b == findpair[half].x);
            return -1;
        }
        else if (findpair[middle].x < b)
            start = middle + 1;
        else
            end = middle - 1;
    }
    return -1;
}
void reorderedge(int start, int end)
{
    int i;
    int middle;
    int a;
    int b;
    int v;
    if (start < end)
    {
        v = findpair[start].x;
        middle = start;
        a = 0;
        b = 0;
        for (i = start + 1; i <= end; i++)
        {
            if (findpair[i].x < v) {
                middle++;
                
                //update findpair[i] and the findpair[middle]
                a = findpair[i].x;
                b = findpair[i].y;
                findpair[i].x = findpair[middle].x;
                findpair[i].y = findpair[middle].y;
                findpair[middle].x = a;
                findpair[middle].y = b;
            }
            
        }
        a = findpair[start].x;
        b = findpair[start].y;
        findpair[start].x = findpair[middle].x;
        findpair[start].y = findpair[middle].y;
        findpair[middle].x = a;
        findpair[middle].y = b;
        halfCompute(start, middle);
        halfCompute(middle + 1, end);
    }
    
}

//==================================================================================================================================
void halfEdgeStore()
{
    int i, j, a, b;
    int startVert, endVert;
    int facenormalnum;
    
    
    Vertex = new HE_vert[vnumb];
    Face = new HE_face[fnumb];
    Edge = new HE_edge[3 * fnumb];
    findpair = new HE_pair[3 * fnumb];
    
    Vertex3f vector1, vector2;
    HE_edge* outgoing_he;
    HE_edge* curr;
    
    
    for (i = 0; i < vnumb; i++)
    {
        Vertex[i].x = vertexarr[i].x;
        Vertex[i].y = vertexarr[i].y;
        Vertex[i].z = vertexarr[i].z;
        Vertex[i].edge = NULL;
    }
    
    for (i = 0; i < fnumb; i++)
    {
        Face[i].edge = NULL;
    }
    for (i = 0; i < fnumb; i++)
    {
        Face[i].edge = &Edge[i * 3];
        for (j = 0; j <= 2; j++)
        {
            startVert = facearr[i].vertoffa[j] - 1;
            endVert = facearr[i].vertoffa[(j + 1) % 3] - 1;
            if (!Vertex[startVert].edge)
            {
                Vertex[startVert].edge = &Edge[3 * i + j];
            }
            
            Edge[3 * i + j].vert = &Vertex[startVert];
            Edge[3 * i + j].face = &Face[i];
            Edge[3 * i + j].prev = &Edge[(3 * i) + ((j + 2) % 3)];
            Edge[3 * i + j].next = &Edge[(3 * i) + ((j + 1) % 3)];
            Edge[3 * i + j].pair = NULL;
            findpair[3 * i + j].x = startVert;
            findpair[3 * i + j].y = endVert;
        }
    }
    reorderedge(0, 3 * fnumb - 1);
    
    for (i = 0; i <(3 * fnumb); i++)
    {
        a = findpair[i].x;
        b = findpair[i].y;//change starting vertex
        if (!Edge[i].pair)
        {
            j = halfCompute(b, a);
            //give the address to the twins edge
            if(j!=-1)
            {  Edge[i].pair = &Edge[j];
                Edge[j].pair = &Edge[i];}
        }
    }
    
    delete findpair;
    
    HE_vert *vert;
    
    
    for (i = 0; i < vnumb; i++)
    {
        vert = &Vertex[i];
        vert->vnormalx = 0;
        vert->vnormaly = 0;
        vert->vnormalz = 0;
        vert->num = 0;
    }
    
    
    HE_vert edge1, edge2, edge3;
    HE_edge *currentedge;
    
    float vectoredge12[3];
    float vectoredge13[3];
    float fsqrtn;
    for (i = 0; i < vnumb; i++)
    {
        vert = &Vertex[i];
        vert->vnormalx = 0;
        vert->vnormaly = 0;
        vert->vnormalz = 0;
        vert->num = 0;
    }
    for (i = 0; i < fnumb; i++)
    {
        currentedge = Face[i].edge;
        
        edge1.x = currentedge->vert->x;
        edge1.y = currentedge->vert->y;
        edge1.z = currentedge->vert->z;
        
        edge2.x = currentedge->next->vert->x;
        edge2.y = currentedge->next->vert->y;
        edge2.z = currentedge->next->vert->z;
        
        edge3.x = currentedge->prev->vert->x;
        edge3.y = currentedge->prev->vert->y;
        edge3.z = currentedge->prev->vert->z;
        
        // two vectors boardering the face
        vectoredge12[0] = edge2.x - edge1.x;
        vectoredge12[1] = edge2.y - edge1.y;
        vectoredge12[2] = edge2.z - edge1.z;
        
        vectoredge13[0] = edge3.x - edge1.x;
        vectoredge13[1] = edge3.y - edge1.y;
        vectoredge13[2] = edge3.z - edge1.z;
        
        //do cross product
        currentedge->face->fnormalx = vectoredge12[1] * vectoredge13[2] - vectoredge13[1] * vectoredge12[2];
        currentedge->face->fnormaly = vectoredge12[2] * vectoredge13[0] - vectoredge13[2] * vectoredge12[0];
        currentedge->face->fnormalz = vectoredge12[0] * vectoredge13[1] - vectoredge13[0] * vectoredge12[1];
        fsqrtn = sqrt(currentedge->face->fnormalx*currentedge->face->fnormalx + currentedge->face->fnormaly*currentedge->face->fnormaly + currentedge->face->fnormalz*currentedge->face->fnormalz);
        
        currentedge->face->fnormalx = currentedge->face->fnormalx / fsqrtn;
        currentedge->face->fnormaly = currentedge->face->fnormaly / fsqrtn;
        currentedge->face->fnormalz = currentedge->face->fnormalz / fsqrtn;
        
        currentedge->vert->vnormalx += currentedge->face->fnormalx;
        currentedge->next->vert->vnormalx += currentedge->face->fnormalx;
        currentedge->prev->vert->vnormalx += currentedge->face->fnormalx;
        
        currentedge->vert->vnormaly += currentedge->face->fnormaly;
        currentedge->next->vert->vnormaly += currentedge->face->fnormaly;
        currentedge->prev->vert->vnormaly += currentedge->face->fnormaly;
        
        currentedge->vert->vnormalz += currentedge->face->fnormalz;
        currentedge->next->vert->vnormalz += currentedge->face->fnormalz;
        currentedge->prev->vert->vnormalz += currentedge->face->fnormalz;
        
        //count the number adjacent faces each vertex has
        currentedge->vert->num += 1;
        currentedge->next->vert->num += 1;
        currentedge->prev->vert->num += 1;
    }
    for (i = 0; i < vnumb; i++)
    {
        vert = &Vertex[i];
        
        
        vert->vnormalx = vert->vnormalx / vert->num;
        vert->vnormaly = vert->vnormaly / vert->num;
        vert->vnormalz = vert->vnormalz / vert->num;
    }
    
}

//==================================================================================================================================

void drawGround(void)
{
				int i;
    
    glPushMatrix();
    glDisable(GL_LIGHTING);
    glLineWidth(0.1);
    for (i = -10; i <= 10; i++)
    {
        glBegin(GL_LINES);
        glColor3f(1.0, 1.0, 1.0);
        glVertex3f(i, -10, 0);
        glColor3f(1.0, 1.0, 1.0);
        glVertex3f(i, 10, 0);
        glEnd();
        glBegin(GL_LINES);
        glColor3f(1.0, 1.0, 1.0);
        glVertex3f(-10, i, 0);
        glColor3f(1.0, 1.0, 1.0);
        glVertex3f(10, i, 0);
        glEnd();
    }
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    
  
}

//==================================================================================================================================

void drawCoordinate(void)
{
    
    GLUquadricObj *objCylinder;
    objCylinder = gluNewQuadric();
    
    
    
    glPushMatrix();
    glColor3f(1.0, 0, 0);
    glTranslatef(0, 0, 1);
    glutSolidCone(0.05, 0.2, 5, 5);
    glPopMatrix();
    gluCylinder(objCylinder, 0.01, 0.01, 1, 32, 5);
    
    
    glPushMatrix();
    glColor3f(0, 1.0, 0);
    glTranslatef(1, 0, 0);
    glRotatef(90, 0, 1, 0);
    glutSolidCone(0.05, 0.2, 5, 5);
    glPopMatrix();
    glRotatef(90, 0, 1, 0);
    gluCylinder(objCylinder, 0.01, 0.01, 1, 32, 5);
    
    glPushMatrix();
    glColor3f(0, 0, 1.0);
    glTranslatef(0, 1, 0);
    glRotatef(-90, 1, 0, 0);
    glutSolidCone(0.05, 0.2, 5, 5);
    glPopMatrix();
    glRotatef(-90, 1, 0, 0);
    gluCylinder(objCylinder, 0.01, 0.01, 1, 32, 5);
    
   
}
//==================================================================================================================================

void drawBoundingBox()
{
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINE_LOOP);//left
    glVertex3f(xmin, ymin, zmin);
    glVertex3f(xmax, ymin, zmin);
    
    glVertex3f(xmax, ymin, zmax);
    glVertex3f(xmin, ymin, zmax);
    glEnd();
    
    glBegin(GL_LINE_LOOP);//right
    glVertex3f(xmin, ymax, zmax);
    glVertex3f(xmax, ymax, zmax);
    glVertex3f(xmax, ymax, zmin);
    
    glVertex3f(xmin, ymax, zmin);
    glVertex3f(xmin, ymax, zmax);
    
    glEnd();
    glBegin(GL_LINE_LOOP);//front
    glVertex3f(xmax, ymin, zmin);
    glVertex3f(xmax, ymax, zmin);
    glVertex3f(xmax, ymax, zmax);
    glVertex3f(xmax, ymin, zmax);
    glVertex3f(xmax, ymin, zmin);
    glEnd();
    
    glBegin(GL_LINE_LOOP);//back
    glVertex3f(xmin, ymin, zmin);
    glVertex3f(xmin, ymax, zmin);
    glVertex3f(xmin, ymax, zmax);
    glVertex3f(xmin, ymin, zmax);
    glVertex3f(xmin, ymin, zmin);
    glEnd();
}

//==================================================================================================================================
void boundingBox()
{
    int i;
    float a, b, c;
    
    xmax = vertexarr[0].x;
    ymax = vertexarr[0].y;
    zmax = vertexarr[0].z;
    
    xmin = vertexarr[0].x;
    ymin = vertexarr[0].y;
    zmin = vertexarr[0].z;
    
    for (i = 0; i < vnumb; i++)
    {
        if (xmax < vertexarr[i].x)
        {
            xmax = vertexarr[i].x;
        }
        if (ymax < vertexarr[i].y)
        {
            ymax = vertexarr[i].y;
        }
        if (zmax < vertexarr[i].z)
        {
            zmax = vertexarr[i].z;
        }
        if (xmin > vertexarr[i].x)
        {
            xmin = vertexarr[i].x;
        }
        if (ymin > vertexarr[i].y)
        {
            ymin = vertexarr[i].y;
        }
        if (zmin > vertexarr[i].z)
        {
            zmin = vertexarr[i].z;
        }
    }
    center.x = (xmax - xmin) / 2 + xmin;
    center.y = (ymax - ymin) / 2 + ymin;
    center.z = (zmax - zmin) / 2 + zmin;
    
    xmax = xmax - center.x;
    ymax = ymax - center.y;
    zmax = zmax - center.z;
    xmin = xmin - center.x;
    ymin = ymin - center.y;
    zmin = zmin - center.z;
    
    a = (xmax - xmin);//length, width and height
    b = (ymax - ymin);
    c = (zmax - zmin);
    
    
    //for viewing object in a normal way
    if (a > b)
    {
        if (a > c)
            scale = 1 / a;
        else
            scale = 1 / c;
    }
    else
    {
        if (b > c)
            scale = 1 / b;
        else
            scale = 1 / c;
    }
}

//==================================================================================================================================
void putBackToOrigin()
{
    int i;
    
    for (i = 0; i < vnumb; i++)
    {
        vertexarr[i].x = vertexarr[i].x - center.x;
        vertexarr[i].y = vertexarr[i].y - center.y;
        vertexarr[i].z = vertexarr[i].z - center.z;
    }
}

//==================================================================================================================================
void scaleObject()
{
    int i;
    for (i = 0; i < vnumb; i++)
    {
        vertexarr[i].x = vertexarr[i].x * scale;
        vertexarr[i].y = vertexarr[i].y * scale;
        vertexarr[i].z = vertexarr[i].z * scale;
    }
    xmax = xmax*scale;//scale bounding box to fit
    ymax = ymax*scale;
    zmax = zmax*scale;
    xmin = xmin*scale;
    ymin = ymin*scale;
    zmin = zmin*scale;
}

//==================================================================================================================================
int readFile(string filename)
{
    int lineline = 0;
    vnumb = 0;
    fnumb = 0;
    
    
    char delims[] = " ";
    
    string line;
    string s;
    
    ifstream input;
    input.open(filename);
    if (!input)
        printf("The file <%s> can not be opened.\n", filename.c_str());
    
    //clear all lines starting with #
    while (!input.eof())
    {
        char content[1000];
        input.getline(content, 1000);
        lineline++;
    }
    input.close();
    //initialize array
    if (vertexarr)
    {
        delete vertexarr;
    }
    if (facearr)
        
    {
        delete facearr;
    }
    
    vertexarr = new Vertex3f[lineline];
    facearr = new Face3i[lineline];
    
    //convert string to char
    
    input.open(filename);
    while (!input.eof())
    {
        const char *token[8] = {};
        char content[1000];
        input.getline(content, 1000);
        int a = 0, i;
        token[0] = strtok(content, delims);
        if (token[0])
        {
            for (a = 1; a < 8; a++)
            {
                token[a] = strtok(NULL, delims);
                if (!token[a])
                    break;
            }
            if (strcmp(token[0], "#") != 0)
            {
                if (strcmp(token[0], "Vertex") == 0)
                {
                    i = atoi(token[1]) - 1;
                    vertexarr[i].x = atof(token[2]);
                    vertexarr[i].y = atof(token[3]);
                    vertexarr[i].z = atof(token[4]);
                    vnumb++;
                }
                if (strcmp(token[0], "Face") == 0)
                {
                    i = atoi(token[1]) - 1;
                    facearr[i].vertoffa[0] = atoi(token[2]);
                    facearr[i].vertoffa[1] = atoi(token[3]);
                    facearr[i].vertoffa[2] = atoi(token[4]);
                    fnumb++;
                }
            }
        }
        
    }
    boundingBox();
    putBackToOrigin();
    scaleObject();
    halfEdgeStore();
    
    return 1;
}
//==================================================================================================================================
void mykey(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'l':
            cout << "key 'l' is pressed! draw the object in prespective projection" << endl;
            projection = 0;
            break;
        case 'o':
            cout << "key 'o' is pressed! draw the object in orthogonal projection" << endl;
            projection = 1;
            break;
        case 'w':
            cout << "key 'w' is pressed! draw the object in wireframe" << endl;
            obj_mode = OBJ_WIREFRAME;
            break;
        case 'p':
            cout << "key 'p' is pressed! draw the object in point" << endl;
            obj_mode = OBJ_POINTS;
            break;
        case 's':
            cout << "key 's' is pressed! draw the object in smooth" << endl;
            obj_mode = OBJ_SMOOTH;
            break;
        case 'f':
            cout << "key 'f' is pressed! draw the object in flat" << endl;
            obj_mode = OBJ_FLAT;
            break;
        case'b':
            cout << "key 'b' is pressed! draw bounding box" << endl;
            bounding_box = 1;
            break;
            
        case '1':
            cout << "draw bunny" << endl;
            fname = "bunny.m";
            break;
        case '2':
            cout << "draw cap" << endl;
            fname = "cap.m";
            break;
        case '3':
            cout << "draw eight" << endl;
            fname = "eight.m";
            break;
        case '4':
            cout << "draw knot" << endl;
            fname = "knot.m";
            break;
        case '5':
            cout << "draw gargoyle" << endl;
            fname = "gargoyle.m";
            break;
    }
    
    // force the redraw function
    glutPostRedisplay();
    
}

//==================================================================================================================================
void reshape(int x, int y)
{
    
    glViewport(0, 0, x, y);
    
    glutPostRedisplay();
}

void render(int obj_mode)
{
    int i;
    if (obj_mode == OBJ_WIREFRAME)
    {
        for (i = 0; i < fnumb; i++)
        {
            glBegin(GL_LINE_LOOP);
            
            glColor3f(1, 0, 1);
            glVertex3f(Vertex[facearr[i].vertoffa[0] - 1].x, Vertex[facearr[i].vertoffa[0] - 1].y, Vertex[facearr[i].vertoffa[0] - 1].z);
            glVertex3f(Vertex[facearr[i].vertoffa[1] - 1].x, Vertex[facearr[i].vertoffa[1] - 1].y, Vertex[facearr[i].vertoffa[1] - 1].z);
            glVertex3f(Vertex[facearr[i].vertoffa[2] - 1].x, Vertex[facearr[i].vertoffa[2] - 1].y, Vertex[facearr[i].vertoffa[2] - 1].z);
            glEnd();
        }
    }
    else if (obj_mode == OBJ_POINTS)
    {
        for (i = 0; i < vnumb; i++)
        {
            glBegin(GL_POINTS);
            glColor3f(1, 0, 0);
            glVertex3f(Vertex[i].x, Vertex[i].y, Vertex[i].z);
            glEnd();
        }
    }
    else if (obj_mode == OBJ_FLAT)
    {
        HE_edge *curr;
        for (i = 0; i < fnumb; i++)
        {
            glShadeModel(GL_FLAT);
            glBegin(GL_TRIANGLES);
            glColor3f(1, 1, 1);
            
            glNormal3f(Face[i].fnormalx, Face[i].fnormaly, Face[i].fnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[0] - 1].x, Vertex[facearr[i].vertoffa[0] - 1].y, Vertex[facearr[i].vertoffa[0] - 1].z);
            
            glNormal3f(Face[i].fnormalx, Face[i].fnormaly, Face[i].fnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[1] - 1].x, Vertex[facearr[i].vertoffa[1] - 1].y, Vertex[facearr[i].vertoffa[1] - 1].z);
            
            
            glNormal3f(Face[i].fnormalx, Face[i].fnormaly, Face[i].fnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[2] - 1].x, Vertex[facearr[i].vertoffa[2] - 1].y, Vertex[facearr[i].vertoffa[2] - 1].z);
            glEnd();
        }
    }
    else
        /*if (obj_mode == OBJ_SMOOTH)*/
    {
        HE_edge *curr;
        glPushMatrix();
        for (i = 0; i < fnumb; i++)
        {
            glShadeModel(GL_SMOOTH);
            glBegin(GL_POLYGON);
            glColor3f(0.7, 0.7, 0.7);
            glNormal3f(Vertex[facearr[i].vertoffa[0] - 1].vnormalx, Vertex[facearr[i].vertoffa[0] - 1].vnormaly, Vertex[facearr[i].vertoffa[0] - 1].vnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[0] - 1].x, Vertex[facearr[i].vertoffa[0] - 1].y, Vertex[facearr[i].vertoffa[0] - 1].z);
            
            
            glNormal3f(Vertex[facearr[i].vertoffa[1] - 1].vnormalx, Vertex[facearr[i].vertoffa[1] - 1].vnormaly, Vertex[facearr[i].vertoffa[1] - 1].vnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[1] - 1].x, Vertex[facearr[i].vertoffa[1] - 1].y, Vertex[facearr[i].vertoffa[1] - 1].z);
            
            
            glNormal3f(Vertex[facearr[i].vertoffa[2] - 1].vnormalx, Vertex[facearr[i].vertoffa[2] - 1].vnormaly, Vertex[facearr[i].vertoffa[2] - 1].vnormalz);
            glVertex3f(Vertex[facearr[i].vertoffa[2] - 1].x, Vertex[facearr[i].vertoffa[2] - 1].y, Vertex[facearr[i].vertoffa[2] - 1].z);
            glEnd();
        }glPopMatrix();
    }
}

//==================================================================================================================================
void display()
{
    readFile(fname);
    
    glClearColor(0.8, 0.8, 0.8, 0.8);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    
    if (projection == 0)
    {
        gluPerspective(140, 1, 0.1, 100);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

        
    }
    else
    {
        glOrtho(-2.0, 2.0, -2.0, 2.0, 0, 50);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(7, 3, 5, 0, 0, 0, 0, 1, 0);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    
   // if (scale_size < 0.01)
    //    scale_size = 0.01;
    
    glTranslatef(x_tran, y_tran, z_tran);
    glRotatef(x_angle, 1.0, 0.0, 0.0);
       glRotatef(y_angle, 0.0, 1.0, 0.0);
    glRotatef(z_angle, 0.0, 0.0, 1.0);
    glScalef(scale_size, scale_size, scale_size);
    
    
    if (draw_coordinate)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glShadeModel(GL_SMOOTH);
        drawCoordinate();
    }
    if (bounding_box == 1)
        
    {
        drawBoundingBox();
    }
    
    
    
    if (obj_mode == OBJ_WIREFRAME)
    {
        render(OBJ_WIREFRAME);
    }
    else if (obj_mode == OBJ_POINTS)
    {
        render(OBJ_POINTS);
    }
    else if (obj_mode == OBJ_FLAT)
    {
        render(OBJ_FLAT);
    }
    else if (obj_mode == OBJ_SMOOTH)
    {
        render(OBJ_SMOOTH);
    }
    else
    {
        render(OBJ_SMOOTH);
    }
    if (draw_ground)
    {
        drawGround();
    }

    glutSwapBuffers();
}


//==================================================================================================================================

void SampleListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
}
void SampleListener::onDisconnect(const Controller& controller) {
    // Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
    std::cout << "Exited" << std::endl;
}
void SampleListener::onFrame(const Controller& controller)
{
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    std::cout << "Frame id: " << frame.id()
    << ", timestamp: " << frame.timestamp()
    << ", hands: " << frame.hands().count()
    << ", extended fingers: " << frame.fingers().extended().count()
    << ", tools: " << frame.tools().count()
    << ", gestures: " << frame.gestures().count() << std::endl;
    
    HandList hands = frame.hands();
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        if(frame.hands().count()==1)
        {
        // Get the first hand
        const Hand hand = *hl;
        int handType = hand.isLeft() ? 1 : 2;
    /*    std::cout << "handtype: "<< handType << ", id: " << hand.id()
        << ", palm position: " << hand.palmPosition() << std::endl;*/
        // Get the hand's normal vector and direction
        const Vector normal = hand.palmNormal();
        const Vector direction = hand.direction();
        std::cout << std::string(2, ' ') <<  "pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
        << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
        << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" <<" extended fingers: " << frame.fingers().extended().count()<< std::endl;
        
            
            //rotate motion->left hand
        if(handType==1)
        {
            if(frame.fingers().extended().count()>4)
            {
             if(frame.id()>prev_frame)
             {
                  prev_frame=frame.id();
                 new_x_angle=direction.pitch() * RAD_TO_DEG;
                 new_y_angle=normal.roll() * RAD_TO_DEG ;
                 new_z_angle=direction.yaw() * RAD_TO_DEG ;
                 if((new_x_angle-prev_x_angle)>5||(new_x_angle-prev_x_angle)<-5)
                     x_angle+=0.3*(new_x_angle-prev_x_angle);
                 
                 if((new_y_angle-prev_y_angle)>5||(new_y_angle-prev_y_angle)<-5)
                     y_angle+=0.3*(new_y_angle-prev_y_angle);
                 
                 if((new_z_angle-prev_z_angle)>5||(new_z_angle-prev_z_angle)<-5)
                     z_angle+=0.3*(new_z_angle-prev_z_angle);
                 
                 prev_x_angle=direction.pitch() * RAD_TO_DEG;
                 prev_y_angle=normal.roll() * RAD_TO_DEG ;
                 prev_z_angle=direction.yaw() * RAD_TO_DEG ;
             }
            }
        }
        
        //translate motion->right hand
       if(handType==2)
       {
       
           if(frame.fingers().extended().count()>4)
           {
               if(frame.id()>prev_frame)
           {
               
           prev_frame=frame.id();
          
               new_x=hand.palmPosition()[0];
               new_y=hand.palmPosition()[1];
               new_z=hand.palmPosition()[2];
               if((new_x-prev_x)>5||(new_x-prev_x)<-5)
                 
                       x_tran+=0.005*(new_x-prev_x);
               
              
               if((new_y-prev_y)>5||(new_y-prev_y)<-5)
                   
                            y_tran+=0.005*(new_y-prev_y);
                             
           
               if((new_z-prev_z)>5||(new_z-prev_z)<-5)
                
                         z_tran+=0.005*(new_z-prev_z);
              
               else if((new_z-prev_z)>20||(new_z-prev_z)<-20)
                   z_tran+=0;
               
              
           prev_x=hand.palmPosition()[0];
           prev_y=hand.palmPosition()[1];
           prev_z=hand.palmPosition()[2];
  
           }
      
           }
      
            
            
          /*  else if(frame.fingers().extended().count()>0&&frame.fingers().extended().count()<3)
            {
                if((frame.id()-prev_frame)>5)
                {
                 prev_frame=frame.id();
                    
                new_scale=hand.palmPosition()[0];
                    if((new_scale-prev_scale)>5||(new_scale-prev_scale)<-5)
                {
                    scale_size*=0.01*(new_scale-prev_scale)+1;
                }
                 
                    
                prev_scale=hand.palmPosition()[0];
                }
            }*/
        }
        }
        
    
        else if(frame.hands().count()==2)
        {
             const Hand hand1 = *hl;
            const Hand hand2=*(hl++);
            if(frame.fingers().extended().count()>8)
            { //hand1 is left hand
            if(hand1.isLeft())
            {
            if(frame.id()>prev_frame)
             {
                   prev_frame=frame.id();
                 new_scale_1=hand1.palmPosition()[0];
                 new_scale_2=hand2.palmPosition()[0];
                 if(fabs(new_scale_1-prev_scale_1)>5)
                     scale_size*=0.005*(new_scale_1-prev_scale_1+new_scale_2-prev_scale_2)+1;
                 prev_scale_1=hand1.palmPosition()[0];
                 prev_scale_2=hand2.palmPosition()[0];
             }
            }
            else
                if(frame.id()>prev_frame)
                {
                      prev_frame=frame.id();
                    new_scale_1=hand1.palmPosition()[0];
                    new_scale_2=hand2.palmPosition()[0];
                    if(fabs(new_scale_2-prev_scale_2)>5)
                        scale_size*=0.005*(new_scale_2-prev_scale_2+new_scale_1-prev_scale_1)+1;
                    prev_scale_1=hand1.palmPosition()[0];
                    prev_scale_2=hand2.palmPosition()[0];
                }
            }

            
        
       
        }
        
    }
    
    // Get tools
    const ToolList tools = frame.tools();
    for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl) {
        const Tool tool = *tl;
        std::cout << std::string(2, ' ') <<  "Tool, id: " << tool.id()
        << ", position: " << tool.tipPosition()
        << ", direction: " << tool.direction() << std::endl;
    }
    
    // Get gestures

    if (!frame.hands().isEmpty() ) {
        std::cout << std::endl;
    }
      glutPostRedisplay();
}


void SampleListener::onFocusGained(const Controller& controller) {
    std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
    std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
    std::cout << "Device Changed" << std::endl;
    const DeviceList devices = controller.devices();
    
    for (int i = 0; i < devices.count(); ++i) {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}
//==================================================================================================================================



void SampleListener::onServiceConnect(const Controller& controller) {
    std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
    std::cout << "Service Disconnected" << std::endl;
}

//==================================================================================================================================
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("LU Jinghan's Mesh Viewer");
 
   
    SampleListener listener;
    Controller controller;
      controller.addListener(listener);
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
  
  
    
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
    
    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    
    glEnable(GL_DEPTH_TEST);
    
    
    glutMainLoop();
    return 1;
}

