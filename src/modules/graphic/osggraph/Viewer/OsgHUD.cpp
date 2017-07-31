/***************************************************************************

    file        : OsgHUD.cpp
    created     : Sun Nov 23 20:12:19 CEST 2014
    copyright   : (C) 2014 by Xavier Bertaux
    email       : Xavier Bertaux
    version     : $Id$
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/RenderInfo>
#include <osg/Projection>

#include <osgDB/WriteFile>

#include <osgText/Text>

#include <osg/TextureRectangle>
#include <osgDB/ReadFile>
#include <osg/TexMat>

#include <osg/BlendFunc>

#include "OsgHUD.h"
#include "OsgMain.h"
#include "tgfclient.h"

#include <sstream> 
#include <iomanip> //setprecision

std::map<std::string,osgText::Text* > hudTextElements;

osg::Vec3 calculatePosition(osg::BoundingBox mybb, std::string objPoint, 
osg::BoundingBox bb,  std::string referenceObjPoint,
float verticalModifier, float horizontalModifier){
	/*
	Possible positioning values:
	tl  //top left
	tr  //tom right
	tc  //top center
	bl  //bottom left
	br  //bottom right
	bc  //bottom center
	ml  //middle left
	mr  //middle right
	mc  //middle center
	*/

	float vPoint=0;
	float hPoint=0;
	float vSign = 0;

	//my starting point
	osg::Vec3 position = osg::Vec3(0.0f,0.0f,0.0f);

	//ref object 
	//vertical
	if(referenceObjPoint.find("t")==0){
		vPoint += bb.yMax();
		vSign = 1;
	}else if(referenceObjPoint.find("b")==0){
		vPoint += bb.yMin();
		vSign = -1;
	}else if(referenceObjPoint.find("m")==0){
		vPoint += (bb.yMax() - bb.yMin())/2;
		vSign = 1;
	}

	//horizontal
	if(referenceObjPoint.find("l")==1){
		hPoint += bb.xMin();
	}else if(referenceObjPoint.find("r")==1){
		hPoint += bb.xMax();
	}else if(referenceObjPoint.find("c")==1){
		hPoint += (bb.xMax() - bb.xMin())/2;
	}


	//my obj /*todo check medium vertical alignment*/
	//vertical
	if(objPoint.find("t")==0){
		vPoint -= (mybb.yMax() - mybb.yMin()) * vSign;//height
	}else if(objPoint.find("b")==0){
		//do nothing
	}else if(objPoint.find("m")==0){
		vPoint -= (mybb.yMax() - mybb.yMin()) * vSign/2;
	}

	//horizontal
	if(objPoint.find("l")==1){
		//nothing to do
	}else if(objPoint.find("r")==1){
		hPoint -= (mybb.xMax() - mybb.xMin());//width
	}else if(objPoint.find("c")==1){
		hPoint -= (mybb.xMax() - mybb.xMin())/2;
	}

	//modifier
	hPoint += horizontalModifier;
	vPoint += verticalModifier;

	// apply the modifiers
	position += osg::Vec3(hPoint,vPoint,0.0f);

	return position;
}

OSGPLOT::OSGPLOT( float positionX,
					float positionY,
					float width,
					float height,
					float maxValue,
					float minValue,
					float timeFrame,
					float referenceLineAtValue,
					std::string Xdata,
					std::string Ydata)
{
	//initialize variables
	this->positionX = positionX;
	this->positionY = positionY;
	this->width = width;
	this->height = height;
	this->maxValue = maxValue;
	this->minValue = minValue;
	this->timeFrame = timeFrame;
	this->referenceLineAtValue = referenceLineAtValue;
	this->Xdata = Xdata;
	this->Ydata = Ydata;

	this->osgGroup = new osg::Group;    

	this->osgMainPlotLineGeometry = new osg::Geometry();
	this->osgMainPlotLineVertices =  new osg::Vec3Array(2);

	this->osgReferencePlotLineGeometry = new osg::Geometry();
	this->osgReferencePlotLineVertices =  new osg::Vec3Array(2);

	this->dataPoints =  new osg::Vec3Array(0);

	//draw the background of the chart
	this->drawBackground();

	//prepare the geode for the "osgReferencePlotLine"
	{
		osg::Geode* geode = new osg::Geode;

		// pass the created vertex array to the points geometry object.
		this->osgReferencePlotLineGeometry->setVertexArray(this->osgReferencePlotLineVertices);
		this->osgReferencePlotLineGeometry->setUseDisplayList (false);
		this->osgReferencePlotLineGeometry->setUseVertexBufferObjects(true);
		this->osgReferencePlotLineGeometry->setDataVariance(osg::Object::DYNAMIC); /*?needed?*/

		// set the same color for the reference plot line
		osg::Vec4Array* plotColors = new osg::Vec4Array;
		plotColors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
		this->osgReferencePlotLineGeometry->setColorArray(plotColors, osg::Array::BIND_OVERALL);

		// set the normal
		osg::Vec3Array* normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
		this->osgReferencePlotLineGeometry->setNormalArray(normals, osg::Array::BIND_OVERALL);

		// tell osg to draw our geometry as lines
		this->osgReferencePlotLineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,this->osgReferencePlotLineVertices->size()));

		// disable lighting (light is always on) and enalbe transparency
		osg::StateSet* stateset = osgReferencePlotLineGeometry->getOrCreateStateSet();
		stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
		stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
		stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

		// add the points geometry to the geode.
		geode->addDrawable(this->osgReferencePlotLineGeometry);
		this->osgGroup->addChild(geode);
	}

	//prepare the geode for the "osgMainPlotLine"
	{
		osg::Geode* geode = new osg::Geode;

		// pass the created vertex array to the points geometry object.
		this->osgMainPlotLineGeometry->setVertexArray(this->osgMainPlotLineVertices);
		this->osgMainPlotLineGeometry->setDataVariance(osg::Object::DYNAMIC); /*?needed?*/

		this->osgMainPlotLineGeometry->setUseDisplayList (false);

		// set the same color for the whole plot line
		osg::Vec4Array* plotColors = new osg::Vec4Array;
		plotColors->push_back(osg::Vec4(0.0f,0.0f,0.0f,0.5f));
		this->osgMainPlotLineGeometry->setColorArray(plotColors, osg::Array::BIND_OVERALL);

		// set the normal
		osg::Vec3Array* normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
		this->osgMainPlotLineGeometry->setNormalArray(normals, osg::Array::BIND_OVERALL);

		// tell osg to draw our geometry as lines
		this->osgMainPlotLineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,this->osgMainPlotLineVertices->size()));

		// disable lighting (light is always on) and enalbe transparency
		osg::StateSet* stateset = osgMainPlotLineGeometry->getOrCreateStateSet();
		stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
		stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
		stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

		// add the points geometry to the geode.
		geode->addDrawable(this->osgMainPlotLineGeometry);
		this->osgGroup->addChild(geode);
	}

}


OSGPLOT::~OSGPLOT()
{

}

osg::ref_ptr <osg::Group> OSGPLOT::getGroup()
{
	return (*this->osgGroup).asGroup();
}
void OSGPLOT::update(tSituation *s, const SDFrameInfo* frameInfo,
						const tCarElt *currCar)
{
	//get x value
	float x = 0;
	if(this->Xdata.compare("time") == 0) x = (float)GfTimeClock();

	//get y value
	float y = 0;
	if(this->Ydata.compare("fps") == 0)					y = (float)frameInfo->fInstFps;
	else if(this->Ydata.compare("carspeed") == 0) 		y = (float)currCar->_speed_x * 3.6;
	else if(this->Ydata.compare("fpsavverrange") == 0) 	y = (float)frameInfo->fAvgFps;
	else if(this->Ydata.compare("carbracketemp") == 0) 	y = (float)currCar->_brakeTemp(0);

	//get z value
	float z=(float)0;

	//add the new point
	this->appendDataPoint(x,y,z);

	//redraw
	this->recalculateDrawnPoint();
}
void OSGPLOT::appendDataPoint(float x, float y, float z)
{
	//add the new element (as last of our vector)
	this->dataPoints->push_back(osg::Vec3(x, y, z));

}
void OSGPLOT::recalculateDrawnPoint()
{
	//recalculate main plot line
	{

		//find max and min values for our plot
		//just draw point that are in our range of time
		for(osg::Vec3Array::iterator it = this->dataPoints->begin(); it != this->dataPoints->end(); /*++it*/) {
			if((*it).x() <= (GfTimeClock() - this->timeFrame) || (*it).x() <= 0){
				it = this->dataPoints->erase(it);

			}else{
				//find max
				if ((*it).y() > this->maxValue){
					this->maxValue = (float)(*it).y();
				}
				//find min
				if ((*it).y() < this->minValue){
					this->minValue = (float)(*it).y();
				}
				++it;
			}
		}

		//cicle trounght data point and calculate correct display position
		int counter = 0;
		this->osgMainPlotLineVertices->resize(this->dataPoints->size());

		for(osg::Vec3Array::iterator it = this->dataPoints->begin(); it != this->dataPoints->end(); ++it, counter++) {

			//copy data
			(*this->osgMainPlotLineVertices)[counter].set(
				((float)(*it).x() - (GfTimeClock() - this->timeFrame))* (this->width / this->timeFrame),
				(float)(*it).y(),
				(float)(*it).z()
			);

			//scale to fit plot area
			(*this->osgMainPlotLineVertices)[counter].set(
				(*this->osgMainPlotLineVertices)[counter].x(),
				((*this->osgMainPlotLineVertices)[counter].y() - this->minValue) / (this->maxValue-this->minValue) * this->height,
				(*this->osgMainPlotLineVertices)[counter].z()
			);

			//move to correct position
			(*this->osgMainPlotLineVertices)[counter].set(
				(*this->osgMainPlotLineVertices)[counter].x() + this->positionX,
				(*this->osgMainPlotLineVertices)[counter].y() + this->positionY,
				(*this->osgMainPlotLineVertices)[counter].z()
			);

		}

		//pass the new vertices to the geometry
		this->osgMainPlotLineGeometry->setVertexArray(this->osgMainPlotLineVertices);

		//update the drawing instructions (for the different number of vertices)
		this->osgMainPlotLineGeometry->removePrimitiveSet(0,1);
		this->osgMainPlotLineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,this->osgMainPlotLineVertices->size()));

	}


	//recalculate reference plot line
	{
		// note, anticlockwise ordering.
		osg::Vec3 myCoords[] =
		{
			osg::Vec3(
				this->positionX,
				((this->referenceLineAtValue - this->minValue) / (this->maxValue - this->minValue) * this->height) + this->positionY,
				0.0f
			),
			osg::Vec3(
				this->positionX + this->width,
				((this->referenceLineAtValue - this->minValue) / (this->maxValue-this->minValue) * this->height) + this->positionY,
				0.0f
			),
		};

		int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);

		this->osgReferencePlotLineVertices = new osg::Vec3Array(numCoords,myCoords);

		//tell osg that our vertices has changed
		this->osgReferencePlotLineVertices->dirty();
		this->osgReferencePlotLineGeometry->setVertexArray(this->osgReferencePlotLineVertices);
	}

}

void OSGPLOT::drawBackground()
{
	// create Geometry object
	osg::Geode* geode = new osg::Geode;
	osg::Geometry* bgGeometry = new osg::Geometry();

	// create the vertices of the geometry
	/*
	 * the numbers rappresent the order of the vertices iside the vec array (took me some time to figure out the order was anticlock wise)
	 * x and y are the axes
	 * assuming this square is the screen
	 * 
	 *      4_______3
	 *      |       |
	 *    y |       |
	 *      |       |
	 *      1_______2
	 *          x
	* */
	osg::Vec3 myCoords[] =
	{
		osg::Vec3(this->positionX, this->positionY, 0.0f),
		osg::Vec3(this->positionX + this->width, this->positionY, 0.0f),
		osg::Vec3(this->positionX + this->width, this->positionY + this->height, 0.0f),
		osg::Vec3(this->positionX, this->positionY+this->height, 0.0f),
	};
	int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);
	osg::Vec3Array* vertices = new osg::Vec3Array(numCoords,myCoords);

	// pass the created vertex array to the points geometry object.
	bgGeometry->setVertexArray(vertices);

	// apply the same color to the whole geometry
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,0.5f));
	bgGeometry->setColorArray(colors, osg::Array::BIND_OVERALL);

	// setup normals
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	bgGeometry->setNormalArray(normals, osg::Array::BIND_OVERALL);

	// tell osg to draw this geometry as quads
	bgGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,numCoords));

	// disable lighting (light is always on) and enable transparency
	osg::StateSet* stateset = bgGeometry->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

	// add the points geometry to the geode.
	geode->addDrawable(bgGeometry);

	// add the geode to the graph group
	this->osgGroup->addChild(geode);
}

// TODO[START]: move this to utils? /src/modules/graphic/osggraph/Utils
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss;
	ss.str(s);
	std::string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}


std::string formatLaptime(tdble sec, int sgn) {

	std::ostringstream lapTimeString;

	if(sec < 0){
		lapTimeString << "-";
	}else{
		lapTimeString << "+";
	}

	sec = abs(sec);

	const int m = (int)(sec / 60.0);
	sec -= 60 * m;
	const int s = (int)(sec);
	sec -= s;
	const int ms = (int)floor(sec * 1000.0);

	//minutes
	if( m < 10){
		lapTimeString << "0";
	}
	lapTimeString << m;

	lapTimeString << ":";

	//seconds
	if( s < 10){
		lapTimeString << "0";
	}
	lapTimeString << s;

	lapTimeString << ".";

	//decimals
	if( ms < 100){
		lapTimeString << "0";
	}

	if( ms < 10){
		lapTimeString << "0";
	}
	lapTimeString << ms;

	return lapTimeString.str();
}

void changeImageSize(osg::Geometry *geom, 
						float newSize/*where 1.0 full image size (width or height) and 0.0 no size (width or height)*/,
						std::string resizeFrom/*left|right|top|bottom == this is the place that will be fixed(not modified), the other one ill be moved to fit the new size*/,
						float hudScale)
{

	osg::TextureRectangle* texture;

	//get the texture data of this object
	texture = dynamic_cast<osg::TextureRectangle*>(geom->getStateSet()->getTextureAttribute(0,osg::StateAttribute::TEXTURE));

	//get the image from the texture data
	osg::Image* img;
	img = texture->getImage();

	//get image dimensions
	float imgWidth = img->s() * hudScale;
	float imgHeight = img->t() * hudScale;

	//adapt the geometry
	{
		osg::Vec3Array* vertices = new osg::Vec3Array;
		vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());

		/*
		 * how vertices are arranged:
		 *      3_______2
		 *      |       |
		 *    y |       |
		 *      |       |
		 *      0_______1
		 *          x
		 * 
		 * [vertices(0-3)][0]=x
		 * [vertices(0-3)][1]=y
		* */

		//change the width
		if(resizeFrom =="left"){
			//change right vertices
			(*vertices)[1][0] = (*vertices)[0][0]+(imgWidth * newSize);
			(*vertices)[2][0] = (*vertices)[0][0]+(imgWidth * newSize);
		}else if(resizeFrom =="right"){
			//change left vertices
			(*vertices)[0][0] = (*vertices)[1][0]-imgWidth+(imgWidth * (1.0f-newSize));
			(*vertices)[3][0] = (*vertices)[1][0]-imgWidth+(imgWidth * (1.0f-newSize));
		}else if(resizeFrom =="top"){
			//change bottom vertices
			(*vertices)[0][1] = (*vertices)[2][1]-imgHeight+(imgHeight * (1.0f-newSize));
			(*vertices)[1][1] = (*vertices)[2][1]-imgHeight+(imgHeight * (1.0f-newSize));
		}else if(resizeFrom =="bottom"){
			//change top vertices
			(*vertices)[2][1] = (*vertices)[0][1]+(imgHeight * newSize);
			(*vertices)[3][1] = (*vertices)[0][1]+(imgHeight * newSize);
		}

		vertices->dirty();

		geom->setVertexArray(vertices);
	}

	//adapt the texture
	{
		osg::Vec2Array* texcoords = new osg::Vec2Array(4);

		texcoords = dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(0));

		if(resizeFrom =="left"){
			(*texcoords)[1][0]= newSize;
			(*texcoords)[2][0]= newSize;
		}else if(resizeFrom =="right"){
			(*texcoords)[0][0] = 1.0f - newSize;
			(*texcoords)[3][0] = 1.0f - newSize;
		}else if(resizeFrom =="top"){
			(*texcoords)[0][1] = 1.0f - newSize;
			(*texcoords)[1][1] = 1.0f - newSize;
		}else if(resizeFrom =="bottom"){
			(*texcoords)[2][1] = newSize;
			(*texcoords)[3][1] = newSize;
		}

		geom->setTexCoordArray(0,texcoords);
	}
}
// TODO[END]: move this to utils? /src/modules/graphic/osggraph/Utils


SDHUD::SDHUD()
{
	_cameraHUD = new osg::Camera;
	
	//initialize some vars
	this->startingFuel = 0.0;
	this->remainingFuelForLaps = 0.0f;

	//
	this->laptimeFreezeCountdown = 3.0f;//keep display for x seconds
	this->laptimeFreezeTime = 0.0f;
	this->timeDiffFreezeCountdown = 8.0f;//keep display for x seconds
	this->timeDiffFreezeTime = 0.0f;
	this->oldSector = 0;
	this->oldBestLapTime;
	this->oldBestSplitTime;
	this->oldLapTime;
	this->numberOfSectors = 0;
	this->oldLapNumber = 0;

	this->hudScale = 1.0f;
}

void SDHUD::CreateHUD(int scrH, int scrW)
{
    // create a camera to set up the projection and model view matrices, and the subgraph to draw in the HUD
	camera = new osg::Camera;

	// set the projection matrix
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0,scrW,0,scrH));

	// set the view matrix
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setViewMatrix(osg::Matrix::identity());

	// only clear the depth buffer
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);

	// draw subgraph after main camera view.
	camera->setRenderOrder(osg::Camera::POST_RENDER);

	// we don't want the camera to grab event focus from the viewers main camera(s).
	camera->setAllowEventFocus(false);

	//calculate optimal hud scale (choose the minor scale from the vertical and horizontal scale)
	float scaleH = (float)scrH/1024;
	float scaleW = (float)scrW/1280;

	if(scaleH < scaleW){
		this->hudScale = scaleH; 
	}else{
		this->hudScale = scaleW; 
	}
	GfLogInfo("OSGHUD: Hud Scale is: %f\n", this->hudScale);

	//generate the hud from the relative xml file
	camera->addChild(this->generateHudFromXmlFile(scrH, scrW));

}

void
SDHUD::DispDebug(const tSituation *s, const SDFrameInfo* frame)
{



}  // grDispDebug

void SDHUD::Refresh(tSituation *s, const SDFrameInfo* frameInfo,
						const tCarElt *currCar)
{

	//update all the graphs
	typedef std::map<std::string,OSGPLOT* >::iterator it_type;
	for(it_type iterator = this->plotElements.begin(); iterator != this->plotElements.end(); iterator++) {
		// iterator->first = key
		// iterator->second = value
		iterator->second->update(s,frameInfo,currCar);

	}

//board

	tCarElt *firstAheadCar;
	tCarElt *secondAheadCar;
	tCarElt *firstBehindCar;
	tCarElt *secondBehindCar;

	std::vector<tCarElt *> boardCars;

	// get pointers for previous and behind cars from us
	if (currCar->_pos > 2) {
		secondAheadCar = s->cars[currCar->_pos - 3]; 
		boardCars.push_back(secondAheadCar);  
	}
	if (currCar->_pos > 1) {
		firstAheadCar = s->cars[currCar->_pos - 2];
		boardCars.push_back(firstAheadCar);  
	}
	//always add our car
	boardCars.push_back(s->cars[currCar->_pos-1]);  

	if (currCar->_pos < s->_ncars ) {
		firstBehindCar = s->cars[currCar->_pos + 0];
		boardCars.push_back(firstBehindCar);  
	}

	if (currCar->_pos < (s->_ncars-1) ) {
		secondBehindCar = s->cars[currCar->_pos + 1];
		boardCars.push_back(secondBehindCar);  
	}

	//hide all board slots... we will enable later what we will use
	for (int id = 1; id <= 5; id++){
		std::ostringstream mapKey;
		//hide board number
		mapKey.str("");
		mapKey << "board-player" << id << "-background";
		this->hudImgElements[mapKey.str()]->setNodeMask(0);
		mapKey.str("");
		mapKey << "board-player" << id << "-background-first";
		this->hudImgElements[mapKey.str()]->setNodeMask(0);
		mapKey.str("");
		mapKey << "board-player" << id << "-background-current";
		this->hudImgElements[mapKey.str()]->setNodeMask(0);

		mapKey.str("");
		mapKey << "board-player" << id << "-number";
		hudTextElements[mapKey.str()]->setNodeMask(0);

		mapKey.str("");
		mapKey << "board-player" << id << "-name";
		hudTextElements[mapKey.str()]->setNodeMask(0);

		mapKey.str("");
		mapKey << "board-player" << id << "-timediff";
		hudTextElements[mapKey.str()]->setNodeMask(0);
	} 

	int id = 0;
	for(std::vector<tCarElt *>::iterator car = boardCars.begin(); car != boardCars.end(); ++car) {

		std::ostringstream mapKey;
		id++;

		mapKey.str("");
		mapKey << "board-player" << id << "-background";
		this->hudImgElements[mapKey.str()]->setNodeMask(1);

		//select special background for current and/or first player
		if ((*car) == currCar){
			this->hudImgElements[mapKey.str()]->setNodeMask(0);
			mapKey << "-current";
			this->hudImgElements[mapKey.str()]->setNodeMask(1);
		}else if ((*car)->_pos == 1){
			this->hudImgElements[mapKey.str()]->setNodeMask(0);
			mapKey << "-first";
			this->hudImgElements[mapKey.str()]->setNodeMask(1);
		}


		std::ostringstream position;
		position.str("");
		position << (*car)->_pos;
		mapKey.str("");
		mapKey << "board-player" << id << "-number";
		hudTextElements[mapKey.str()]->setText(position.str());
		hudTextElements[mapKey.str()]->setNodeMask(1);
	  
		//update board names texts
		mapKey.str("");
		mapKey << "board-player" << id << "-name";
		hudTextElements[mapKey.str()]->setText((*car)->_name);
		hudTextElements[mapKey.str()]->setNodeMask(1);

		//update time diff texts
		mapKey.str("");
		mapKey << "board-player" << id << "-timediff";

		if(this->oldLapNumber != currCar->_laps){
			this->timeDiffFreezeTime = GfTimeClock();
		}

		if ( GfTimeClock() < (this->timeDiffFreezeTime + this->timeDiffFreezeCountdown)){
			this->oldLapNumber = currCar->_laps;

			std::ostringstream tempStr;
			tempStr.str("");
			if((*car)->_laps == currCar->_laps){
				tempStr << formatLaptime(((*car)->_curTime - currCar->_curTime),1);
			}else if ((*car)->_laps > currCar->_laps){
				tempStr << ((*car)->_laps - currCar->_laps);
				tempStr << " laps";
			}

			hudTextElements[mapKey.str()]->setText(tempStr.str());
			hudTextElements[mapKey.str()]->setNodeMask(1);
			//hide time diff for our car
			if ((*car) == currCar){
				hudTextElements[mapKey.str()]->setNodeMask(0);
			}
		}else{
			hudTextElements[mapKey.str()]->setNodeMask(0);
		}


	}


//position box

	std::ostringstream temp;
	temp.str("");
	temp << s->_ncars;
	hudTextElements["position-container-totalplayers"]->setText(temp.str());
	temp.str("");
	temp << currCar->_pos;
	hudTextElements["position-container-currentposition"]->setText(temp.str());


//laps box

	temp.str("");
	temp << s->_totLaps;
	hudTextElements["lap-container-totallaps"]->setText(temp.str());
	temp.str("");
	temp << currCar->_laps;
	hudTextElements["lap-container-currentlap"]->setText(temp.str());
	//car_->_deltaBestLapTimefuel


//laptime

	float currentPrevSectorSplitTime = currCar->_curSplitTime[currCar->_currentSector - 1]; // our time in the sector we have "just" run over
	float bestPrevSectorSplitTime = currCar->_bestSplitTime[currCar->_currentSector-1]; // the best split time of the sector we are in this moment
	float bestSplitTime = currCar->_bestSplitTime[currCar->_currentSector]; // the best split time of the sector we are in this moment
	float splitTimeDiff = 0;

	//in the first lap we count how many sector this track have
	//ReInfo->track->numberOfSectors is a better source for this but we have no access to it in this file
	if( this->numberOfSectors < currCar->_currentSector+1){
		this->numberOfSectors = currCar->_currentSector+1;
	}

	if( this->oldSector != currCar->_currentSector){
		this->laptimeFreezeTime = GfTimeClock();
		this->oldSector = currCar->_currentSector;
		if (currCar->_currentSector == 0){
			hudTextElements["laptime-last-time"]->setText(formatLaptime(currCar->_lastLapTime,0));
			if(currCar->_lastLapTime == currCar->_bestLapTime){

				this->hudImgElements["laptime-last-background-green"]->setNodeMask(1);
				this->hudImgElements["laptime-last-background-normal"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-grey"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-violet"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-red"]->setNodeMask(0);

			}else{

				this->hudImgElements["laptime-last-background-normal"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-grey"]->setNodeMask(1);
				this->hudImgElements["laptime-last-background-violet"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-green"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-red"]->setNodeMask(0);

			}

		}else{

			if(currentPrevSectorSplitTime < bestPrevSectorSplitTime){

				this->hudImgElements["laptime-last-background-normal"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-grey"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-violet"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-green"]->setNodeMask(1);
				this->hudImgElements["laptime-last-background-red"]->setNodeMask(0);

			}else{

				this->hudImgElements["laptime-last-background-normal"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-grey"]->setNodeMask(1);
				this->hudImgElements["laptime-last-background-violet"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-green"]->setNodeMask(0);
				this->hudImgElements["laptime-last-background-red"]->setNodeMask(0);

			}
		}
	}

	if(GfTimeClock() > (this->laptimeFreezeTime + this->laptimeFreezeCountdown)){
		//remember our current sector
		this->oldSector = currCar->_currentSector;

		temp.str("");
		temp << "S" << (this->oldSector+1);
		hudTextElements["laptime-sector-description"]->setText(temp.str());

		hudTextElements["laptime-last-time"]->setText(formatLaptime(currCar->_curLapTime,0));

		this->hudImgElements["laptime-last-background-normal"]->setNodeMask(1);
		this->hudImgElements["laptime-last-background-grey"]->setNodeMask(0);
		this->hudImgElements["laptime-last-background-violet"]->setNodeMask(0);
		this->hudImgElements["laptime-last-background-green"]->setNodeMask(0);
		this->hudImgElements["laptime-last-background-red"]->setNodeMask(0);

		//show laptime
		hudTextElements["laptime-best-time"]->setText(formatLaptime(bestSplitTime,0));
		hudTextElements["laptime-last-time"]->setText(formatLaptime(currCar->_curLapTime,0));
		this->oldBestSplitTime = bestSplitTime;
		this->oldBestLapTime = currCar->_bestLapTime;
		this->oldLapTime = currCar->_curLapTime;

		//on the last sector show the totallaptime
		if(currCar->_currentSector == this->numberOfSectors-1){
			hudTextElements["laptime-best-time"]->setText(formatLaptime(currCar->_bestLapTime,0));
		}
	}


//fuel

	temp.str("");
	temp << std::fixed << std::setprecision(1) << currCar->_fuel;
	hudTextElements["fuel-info-quantity-number"]->setText(temp.str());

	temp.str("");

	if (this->startingFuel == 0.0f){
		this->startingFuel = currCar->_fuel;
	}

	//when we have done at least one lap calculate remaining fuel
	if (currCar->_laps < 1){
		float fuelConsumpionPerLap = 0.1f;
	}

	if( currCar->_laps > this->carLaps && currCar->_laps > 1){
		if (currCar->_laps == 2){
			this->lapLength = this->lapLength - currCar->_distRaced;
		}
		float fuelConsumpionPerLap = (this->startingFuel - currCar->_fuel)  / (float)(currCar->_laps-1);
		this->remainingFuelForLaps = currCar->_fuel / fuelConsumpionPerLap;
		this->carLaps = currCar->_laps;

	}

	//if we have fuel for more than one lap display how many
	if(this->remainingFuelForLaps > 0 ){
		temp << std::fixed << std::setprecision(0) << this->remainingFuelForLaps;
	}else{
		temp << "---";
	}

	hudTextElements["fuel-info-laps-number"]->setText(temp.str());

	float carFuel = (float)((float)currCar->_fuel / (float)currCar->_tank);

	//update fuel bar
	changeImageSize(this->hudImgElements["fuel-icon-empty"], 1.0-carFuel, "top", this->hudScale);
	changeImageSize(this->hudImgElements["fuel-icon-full"], carFuel, "bottom", this->hudScale);


//abs
//tcs
//TODO: speed limiter

	bool abs = false;   // Show ABS indicator?
	bool tcs = false;   // Show TCS indicator?
	bool spd = false;   // Show speed limiter indicator?

	// Parse control messages if they include ABS / TCS / SPD
	for (int i = 0; i < 4; i++) {
		if (currCar->ctrl.msg[i]) {
			abs = abs || strstr(currCar->ctrl.msg[i], "ABS");
			tcs = tcs || strstr(currCar->ctrl.msg[i], "TCS");
			spd = spd || strstr(currCar->ctrl.msg[i], "Speed Limiter On");
		}
	}
	this->hudImgElements["abs-icon"]->setNodeMask(abs);
	this->hudImgElements["tcs-icon"]->setNodeMask(tcs);



//gear

	// show gear: "N" for neutral, "R" for retro, "gearnumber" for all the others
	temp.str("");
	if(currCar->_gear == 0){
		temp << "N";
	}else if (currCar->_gear == -1){
		temp << "R";
	}else{
		temp << currCar->_gear;
	}
	hudTextElements["gear-number"]->setText(temp.str());


//speed

	temp.str("");
	//get speed in km/h
	temp << (int)(currCar->_speed_x * 3.6);
	hudTextElements["speed-number"]->setText(temp.str());


//damage

	float carDamage = (float)((currCar->_dammage) / (float)s->_maxDammage);
	changeImageSize(this->hudImgElements["engine-icon"], 1.0-carDamage, "top", this->hudScale);
	changeImageSize(this->hudImgElements["engine-icon-damaged"], carDamage, "bottom", this->hudScale);


//rpm

	float rpmWidth = 1.0 / currCar->_enginerpmMax * currCar->_enginerpm;
	changeImageSize(this->hudImgElements["rpm-on"], rpmWidth, "left", this->hudScale);
	changeImageSize(this->hudImgElements["rpm-off"], 1.0-rpmWidth, "right", this->hudScale);



	//make the camera visible
	_cameraHUD->setNodeMask(1-_cameraHUD->getNodeMask());

}

void SDHUD::ToggleHUD1()
{
    _cameraHUD->setNodeMask(1-_cameraHUD->getNodeMask());
}

osg::ref_ptr <osg::Group> SDHUD::generateHudFromXmlFile(int scrH, int scrW){

	osg::ref_ptr<osg::Group> group = new osg::Group;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	group->addChild(geode);

	// turn lighting off for the text and disable depth test to ensure it's always ontop.
	osg::StateSet* stateset = geode->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	//screen bounding box
	osg::BoundingBox screenBB;
	screenBB.expandBy(osg::Vec3(0.0f,0.0f,0.0f));
	screenBB.expandBy(osg::Vec3(scrW,scrH,0.0f));

	std::string sectionPath= "board";
	std::string subSectionPath = "";
	std::string subSectionName = "";

	std::string configFileUrl= GetLocalDir();
	configFileUrl.append("config/osghudconfig.xml");
	int paramValue = 0;

	//open the file
	void *paramHandle = GfParmReadFile(configFileUrl.c_str(), GFPARM_RMODE_STD);

	//for each section on the
	if (GfParmListSeekFirst(paramHandle, sectionPath.c_str()) == 0) {
		do {
				subSectionName = GfParmListGetCurEltName(paramHandle, sectionPath.c_str());
				subSectionPath = sectionPath + "/" + subSectionName;

				//get a list of the params in this section
				std::vector<std::string> paramsInSection = GfParmListGetParamsNamesList(paramHandle, subSectionPath.c_str());

				std::string type = GfParmGetStr (paramHandle, subSectionPath.c_str(),"type", "no default value" );

				/* ============================
					 CREATE OSG TEXT
				   ============================*/
				if (type == "text" ){

					//read data into local variables
					std::string elementId = 	subSectionName;
					std::string textStr = 		GfParmGetStr (paramHandle, subSectionPath.c_str(),"text", "" );
					std::string fontFileUrl = 	GfParmGetStr (paramHandle, subSectionPath.c_str(),"fontFileUrl", "" );
					std::string colorString = 	GfParmGetStr (paramHandle, subSectionPath.c_str(),"color", "" );
					float fontSize = 			GfParmGetNum (paramHandle, subSectionPath.c_str(),"fontSize", "",0 ) * this->hudScale;
					std::string textAlign = 	GfParmGetStr (paramHandle, subSectionPath.c_str(),"textAlign", "" );

					std::string positionRefObj = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObj", "" );
					std::string positionRefObjPoint = 		GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObjPoint", "tl" );
					std::string positionMyPoint = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-myPoint", "tl" );
					float positionVerticalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-verticalModifier", "",0 ) * this->hudScale;
					float positionHorizontalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-horizontalModifier", "",0 ) * this->hudScale;

					GfLogInfo("OSGHUD: Generate text object: %s \n", elementId.c_str());

					//create the osg::text object
					osgText::Text* text = new  osgText::Text;

					//add the text obj to our pool (we willl need it later)
					hudTextElements[elementId] = text;

					//extract and apply the color
					{
						std::vector<std::string> colorStringVector = split(colorString.c_str(), '#');

						osg::Vec4 color = osg::Vec4(
							std::atof(colorStringVector[0].c_str()),
							std::atof(colorStringVector[1].c_str()),
							std::atof(colorStringVector[2].c_str()),
							std::atof(colorStringVector[3].c_str())
						);

						text->setColor(color);
					}

					//set the font
					{
						std::string fontsMainDirectory = GetDataDir();
						fontsMainDirectory = fontsMainDirectory+"data/fonts";
						fontFileUrl = fontsMainDirectory+fontFileUrl;
						text->setFont(fontFileUrl);

						//font resolution
						text->setFontResolution(200,200);

						//set the font size
						text->setCharacterSize(fontSize);
					}

					//set alignement
					if (textAlign==""){
						text->setAlignment(osgText::Text::LEFT_BOTTOM_BASE_LINE );
					}else if (textAlign=="LEFT_BOTTOM"){
						text->setAlignment(osgText::Text::LEFT_BOTTOM_BASE_LINE );
					}else if (textAlign=="RIGHT_BOTTOM"){
						text->setAlignment(osgText::Text::RIGHT_BOTTOM_BASE_LINE );
					}

					//set the text string
					text->setText(textStr.c_str());

					//set the position
					//find the referenceObj pointer and then get his bounding box
					osg::BoundingBox refObjBb;
					if ( hudTextElements.find(positionRefObj.c_str()) != hudTextElements.end() ) {
						refObjBb = hudTextElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudImgElements.find(positionRefObj.c_str()) != this->hudImgElements.end() ) {
						refObjBb = this->hudImgElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudGraphElements.find(positionRefObj.c_str()) != this->hudGraphElements.end() ) {
						//refObjBb = this->hudGraphElements[positionRefObj.c_str()]->getBoundingBox();
					}else{
						GfLogInfo("OSGHUD: No (valid) reference object given for the current element alignement: Assuming Screen!");
						refObjBb = screenBB;
					}

					//calculate the positioning
					osg::Vec3 position = calculatePosition(text->getBoundingBox(),positionMyPoint,refObjBb,positionRefObjPoint, positionVerticalModifier, positionHorizontalModifier); 

					//asign the position
					text->setPosition(position);

					//TODO: enable shadows for texts?
					//text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_RIGHT); //OUTLINE //DROP_SHADOW_BOTTOM_RIGHT
					/*
					text->setBackdropOffset(0.07f);
					color = osg::Vec4(0.57f,0.57f,0.57f,0.6f);
					text->setBackdropColor(color);
					*/

					//add the text to the hud
					geode->addDrawable( text );    

				}else if( type == "image"){

					/* ============================
						 CREATE OSG IMAGE
					   ============================*/
					//read data into local variables
					std::string elementId = 				subSectionName;
					std::string url = 						GfParmGetStr (paramHandle, subSectionPath.c_str(),"url", "" );

					std::string positionRefObj = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObj", "" );
					std::string positionRefObjPoint = 		GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObjPoint", "tl" );
					std::string positionMyPoint = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-myPoint", "tl" );
					float positionVerticalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-verticalModifier", "",0 ) * this->hudScale;
					float positionHorizontalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-horizontalModifier", "",0 ) * this->hudScale;

					GfLogInfo("OSGHUD: Generate image object: %s \n", elementId.c_str());

					//start preparing the image
					std::string filename = GetDataDir();
					filename = filename+url;

					//get the bounding box
					osg::BoundingBox bb;
					for(unsigned int i=0;i<geode->getNumDrawables();++i)
					{
						bb.expandBy(geode->getDrawable(i)->getBoundingBox());
					}

					//check that the image file exist
					if (!GfFileExists(filename.c_str())){
						GfLogError ("OSGHUD: Specified image file does not exist: %s.\n", filename.c_str());
					}
					osg::Image* img = osgDB::readImageFile(filename);

					//correct the image size to match the hud scale
					float imgWidth = img->s() *  this->hudScale;
					float imgHeight = img->t() * this->hudScale;

					// create geometry
					osg::Geometry* geom = new osg::Geometry;
					this->hudImgElements[elementId] =  geom;

					//set the position
					//find the referenceObj pointer and then get his bounding box
					osg::BoundingBox refObjBb;
					if ( hudTextElements.find(positionRefObj.c_str()) != hudTextElements.end() ) {
						refObjBb = hudTextElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudImgElements.find(positionRefObj.c_str()) != this->hudImgElements.end() ) {
						refObjBb = this->hudImgElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudGraphElements.find(positionRefObj.c_str()) != this->hudGraphElements.end() ) {
						//refObjBb = this->hudGraphElements[positionRefObj.c_str()]->getBoundingBox();
					}else{
						GfLogInfo("OSGHUD: No (valid) reference object given for the current element alignement: Assuming Screen!");
						refObjBb = screenBB;
					}

					//get object bounding box
					osg::BoundingBox myObjBb;
					myObjBb.expandBy(osg::Vec3(0.0f,0.0f,0.0f));
					myObjBb.expandBy(osg::Vec3(imgWidth,imgHeight,0.0f));

					//calculate the positioning
					osg::Vec3 position = calculatePosition(myObjBb,positionMyPoint,refObjBb,positionRefObjPoint, positionVerticalModifier, positionHorizontalModifier); 

					//asign the position
					float positionLeft =   position.x();
					float positionBottom = position.y(); 

					//create the vertices for the image geometry and assign them
					osg::Vec3Array* vertices = new osg::Vec3Array;
					float depth = 0.0f-0.1f;
					vertices->push_back(osg::Vec3( positionLeft			,positionBottom,depth)); //bottom left
					vertices->push_back(osg::Vec3( positionLeft+imgWidth,positionBottom,depth)); //bottom right
					vertices->push_back(osg::Vec3( positionLeft+imgWidth,positionBottom+imgHeight	 ,depth)); //top right
					vertices->push_back(osg::Vec3( positionLeft			,positionBottom+imgHeight	 ,depth)); //topleft
					geom->setVertexArray(vertices);


					// texture the geometry and apply material

					// calculate textures coordinates
					osg::Vec2Array* texcoords = new osg::Vec2Array(4);
					(*texcoords)[0].set(0.0f, 0.0f);
					(*texcoords)[1].set(1.0f, 0.0f);
					(*texcoords)[2].set(1.0f, 1.0f);
					(*texcoords)[3].set(0.0f, 1.0f);
					geom->setTexCoordArray(0,texcoords);

					// calculate normals
					osg::Vec3Array* normals = new osg::Vec3Array(1);
					(*normals)[0].set(0.0f,-1.0f,0.0f);
					geom->setNormalArray(normals);
					geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

					// assign colors
					osg::Vec4Array* colors = new osg::Vec4Array(1);
					(*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
					geom->setColorArray(colors);
					geom->setColorBinding(osg::Geometry::BIND_OVERALL);

					// draw the vertices as quads
					geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

					// disable display list so our modified tex coordinates show up
					geom->setUseDisplayList(false);

					// setup texture
					osg::TextureRectangle* texture = new osg::TextureRectangle;

					texture->setImage(img);

					// setup stateset
					osg::StateSet* state = geom->getOrCreateStateSet();
					state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

					// setup material
					osg::TexMat* texmat = new osg::TexMat;
					texmat->setScaleByTextureRectangleSize(true);
					state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);

					//enable gl_blending (for texture transparency)
					state->setMode(GL_BLEND, osg::StateAttribute::ON);
					osg::BlendFunc* blend = new osg::BlendFunc;
					blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_DST_ALPHA);   

					// turn off lighting (light always on)
					state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

					//add the image geometry to the hud
					geode->addDrawable(geom);

				}else if( type == "graph"){

					/* ============================
						 CREATE OSG GRAPH
					   ============================*/
					//read data into local variables
					std::string elementId = subSectionName;

					//positioning variables
					float posFromTop = 						GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-from-top", "",0 );
					float posFromBottom = 					GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-from-bottom", "",0 );
					float posFromLeft = 					GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-from-left", "",0 );
					float posFromRight = 					GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-from-right", "",0 );
					float posHorizontalCenter = 			GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-horizontal-center", "",0 );

					std::string positionRefObj = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObj", "" );
					std::string positionRefObjPoint = 		GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-refObjPoint", "tl" );
					std::string positionMyPoint = 			GfParmGetStr (paramHandle, subSectionPath.c_str(),"position-myPoint", "tl" );
					float positionVerticalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-verticalModifier", "",0 ) * this->hudScale;
					float positionHorizontalModifier = 		GfParmGetNum (paramHandle, subSectionPath.c_str(),"position-horizontalModifier", "",0 ) * this->hudScale;

					//graph variables
					float width =  							GfParmGetNum (paramHandle, subSectionPath.c_str(),"width", "",0 );
					float height =  						GfParmGetNum (paramHandle, subSectionPath.c_str(),"height", "",0 ); 
					float maxValue =  						GfParmGetNum (paramHandle, subSectionPath.c_str(),"maxValue", "",0 ); 
					float minValue =  						GfParmGetNum (paramHandle, subSectionPath.c_str(),"minValue", "",0 );
					float timeFrame =  						GfParmGetNum (paramHandle, subSectionPath.c_str(),"timeFrame", "",0 );
					float referenceLineAtValue =			GfParmGetNum (paramHandle, subSectionPath.c_str(),"referenceLineAtValue", "",0 );
					std::string Xdata =						GfParmGetStr (paramHandle, subSectionPath.c_str(),"Xdata", "" );
					std::string Ydata =						GfParmGetStr (paramHandle, subSectionPath.c_str(),"Ydata", "" );

					GfLogInfo("OSGHUD: Generate graph object: %s \n", elementId.c_str());

					//calculate position
					//find the referenceObj pointer and then get his bounding box
					osg::BoundingBox refObjBb;
					if ( hudTextElements.find(positionRefObj.c_str()) != hudTextElements.end() ) {
						refObjBb = hudTextElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudImgElements.find(positionRefObj.c_str()) != this->hudImgElements.end() ) {
						refObjBb = this->hudImgElements[positionRefObj.c_str()]->getBoundingBox();
					}else if ( this->hudGraphElements.find(positionRefObj.c_str()) != this->hudGraphElements.end() ) {
						//refObjBb = this->hudGraphElements[positionRefObj.c_str()]->getBoundingBox();
					}else{
						GfLogInfo("OSGHUD: No (valid) reference object given for the current element alignement: Assuming Screen!");
						refObjBb = screenBB;
					}

					//calculate our bounding box
					osg::BoundingBox plotBB;
					plotBB.expandBy(osg::Vec3(0.0f,0.0f,0.0f));
					plotBB.expandBy(osg::Vec3(width,height,0.0f));

					//calculate the positioning
					osg::Vec3 position = calculatePosition(plotBB,positionMyPoint,refObjBb,positionRefObjPoint, positionVerticalModifier, positionHorizontalModifier);

					float positionX = position.x();
					float positionY = position.y();

					//istantiate the graph
					this->plotElements[elementId] = new OSGPLOT(
						positionX,
						positionY,
						width,
						height,
						maxValue,
						minValue,
						timeFrame,
						referenceLineAtValue,
						Xdata.c_str(),
						Ydata.c_str()
					);

					//camera->addChild(testPLot->getGroup());
					//return this->plotElements[elementId]->getGroup();

					group->addChild(this->plotElements[elementId]->getGroup());

				}else{

				/* ============================
					 NO MORE HUD TYPE OPTIONS...??
				   ============================*/
					GfLogInfo("OSG-HUD object type not recognized: %s", type.c_str());
				}

		} while (GfParmListSeekNext(paramHandle, sectionPath.c_str()) == 0);
	}

	//release the xml file
	GfParmReleaseHandle(paramHandle);

	//make the hud visible
	geode->setNodeMask(1-geode->getNodeMask());

	//return the hud object
	return (*group).asGroup();
}


SDHUD::~SDHUD()
{
	//TODO: check we may have something more to clean up
	//do some cleanup
	hudTextElements.clear();
	this->hudImgElements.clear();
	this->hudGraphElements.clear();
}
