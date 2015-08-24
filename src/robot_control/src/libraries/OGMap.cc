#include "OGMap.h"
#include <cmath>
#include <fstream>

using namespace std;

//Override Constructor
OGMap::OGMap(vector<float> dmap, const int width, const double p, const double s){
    unsigned int i, j;

    scale = s;
    proportion = p;

    //Cast the double 1D map to a bool 2D map
    for(i = 0; i < dmap.size()/width; i++){
        vector<bool> aux;
        for(j = 0; j < width; j++)
            aux.push_back(dmap[i*width + j]>0.94);
        map.push_back(aux);
    }


}

OGMap::~OGMap() {
}

int OGMap::getSizeX(){
    return map.size();
}

int OGMap::getSizeY(){
    return map[0].size();
}

double OGMap::getScale(){
    return scale;
}

void OGMap::setScale(const double s){
    scale = s;
}

int OGMap::getProportion(){
    return proportion;
}

void OGMap::setProportion(const int p){
    proportion = p;
}

double OGMap::getOffsetX(){
    return offsetX;
}

void OGMap::setOffsetX(const double ox){
    offsetX = ox;
}

double OGMap::getOffsetY(){
    return offsetY;
}

void OGMap::setOffsetY(const double oy){
    offsetY = oy;
}

std::vector< std::vector<bool> > OGMap::getMap(){
    return map;
}

dpos_t OGMap::itod(const ipos_t ipos){
    dpos_t dpos;
    dpos.x = (double)ipos.j * (double)proportion * scale + scale/2 + offsetX;
    dpos.y = (double)ipos.i * (double)proportion * scale + scale/2 + offsetY;

    return dpos;
}


ipos_t OGMap::dtoi(const dpos_t dpos){
    ipos_t ipos;
    ipos.i = ((dpos.y - offsetY)/scale)/proportion;
    ipos.j = ((dpos.x - offsetX)/scale)/proportion;

    return ipos;
}

bool OGMap::isOccupatedPosition(const ipos_t ipos) throw(int){
    if( (ipos.i < 0) || (ipos.j < 0) || (ipos.i >= getSizeX()) || (ipos.j >= getSizeY()) )
        throw (EXCP_IMP);

    return map[ipos.i][ipos.j] | neighborhood32(ipos);
}

bool OGMap::isOccupatedPosition(const int i, const int j) throw(int){
    if( (i < 0) || (j < 0) || (i >= getSizeX()) || (j >= getSizeY()) )
        throw (EXCP_IMP); 
    
    ipos_t ipos;
    ipos.i = i;
    ipos.j = j;
    //return map[i][j] | neighborhood8(ipos);
    return map[i][j] | neighborhood32(ipos);
}

bool OGMap::neighborhood8(const ipos_t ipos){

    bool occupation = false;
    if(ipos.i > 0){
    	occupation |= map[ipos.i-1][ipos.j];
    }
    if(ipos.i < getSizeX() - 1){
    	occupation |= map[ipos.i+1][ipos.j];
    }
    if(ipos.j > 0){
    	occupation |= map[ipos.i][ipos.j-1];
    }
    if(ipos.j < getSizeY() - 1){
    	occupation |= map[ipos.i][ipos.j+1];
    }
    
    if(ipos.i > 0 && ipos.j > 0){
    	occupation |= map[ipos.i-1][ipos.j-1];
    }
    if((ipos.i < getSizeX() - 1) && ipos.j > 0){
    	occupation |= map[ipos.i+1][ipos.j-1];
    }
    if(ipos.i > 0 && (ipos.j < getSizeY() - 1)){
    	occupation |= map[ipos.i-1][ipos.j+1];
    }
    if((ipos.i < getSizeX() - 1) && (ipos.j < getSizeY() - 1)){
    	occupation |= map[ipos.i+1][ipos.j+1];
    }
    return occupation;
}

bool OGMap::neighborhood16(const ipos_t ipos){
    int i, j;
    bool occupation = false;
    for(i = ipos.i - 2; i <= ipos.i + 2; i++)
        for(j = ipos.j - 2; j <= ipos.j + 2; j++)
            if( i >= 0 && j >= 0 && i < getSizeX() && j < getSizeY())
                occupation |= map[i][j];
    return occupation;
}

bool OGMap::neighborhood32(const ipos_t ipos){
    int i, j;
    bool occupation = false;
    for(i = ipos.i - 3; i <= ipos.i + 3; i++)
        for(j = ipos.j - 3; j <= ipos.j + 3; j++)
            if( i >= 0 && j >= 0 && i < getSizeX() && j < getSizeY())
                occupation |= map[i][j];
    return occupation;    
}

bool OGMap::neighborhood64(const ipos_t ipos){
    int i, j;
    bool occupation = false;
    for(i = ipos.i - 4; i <= ipos.i + 4; i++)
        for(j = ipos.j - 4; j <= ipos.j + 4; j++)
            if( i >= 0 && j >= 0 && i < getSizeX() && j < getSizeY())
                occupation |= map[i][j];
    return occupation;    
}

void OGMap::setPositionOccupation(const ipos_t ipos, bool occupation) throw(int){
    if( (ipos.i < 0) || (ipos.j < 0) || (ipos.i >= getSizeX()) || (ipos.j >= getSizeY()) )
        throw (EXCP_IMP);

    map[ipos.i][ipos.j] = occupation;
}


void OGMap::printOccupationGrid(const char* fileName){
    int i,j;
    ofstream file(fileName);

    for( i=0 ; i<getSizeX() ; i++ ){
        for( j=0 ; j<getSizeY() ; j++ )
            if(isOccupatedPosition(i,j))
                file << "+";
            else
                file << " ";
        file << endl;
    }
    file.close();
}

list<ipos_t> OGMap::getPath(ipos_t iposBegin, ipos_t iposEnd) throw(int){
    
    int i,j;
    //path list using indices
    list<ipos_t> path;
    //list of adjacent nodes that need to be checked out
    list<ipos_t> openList;
    //iterator for the indices
    list<ipos_t>::iterator it;
    //parent list of each node
    ipos_t father[getSizeX()][getSizeY()];
    //list to check if a node is inside the closed list
    bool inClosedList[getSizeX()][getSizeY()];
    //list of the cost to move from initial node to each node  
    int costG[getSizeX()][getSizeY()];
    //list of the estimated cost from each node to the goal node  
    int costH[getSizeX()][getSizeY()];
    //Heuristic cost of each node
    int costF[getSizeX()][getSizeY()];

    // validate begin and end positions
    if( (iposBegin.i < 0) || (iposBegin.j < 0) || (iposBegin.i >= getSizeX()) || (iposBegin.j >= getSizeY()) )
        throw (EXCP_IMP);
    if( (iposEnd.i < 0)   || (iposEnd.j < 0)   || (iposEnd.i >= getSizeX())   || (iposEnd.j >= getSizeY()) )
        throw (EXCP_IMP);


    // initiate the algorithm
    for( i=0 ; i < getSizeX() ; i++ )
        for( j=0 ; j  < getSizeY() ; j++ ){
            inClosedList[i][j] = false;
            costF[i][j] = -1;
        }

    openList.push_back(iposBegin);
    costG[iposBegin.i][iposBegin.j] = 0;
    costH[iposBegin.i][iposBegin.j] = 10 * (abs(iposBegin.i - iposEnd.i) + abs(iposBegin.j - iposEnd.j));
    costF[iposBegin.i][iposBegin.j] = costG[iposBegin.i][iposBegin.j] + costH[iposBegin.i][iposBegin.j];

    // algorithm core
    do{

        // find better cell
        ipos_t betterCell = openList.front();
        it = openList.begin()++;
        while(it != openList.end()){
            if(costF[it->i][it->j] < costF[betterCell.i][betterCell.j]){
                betterCell.i = it->i;
                betterCell.j = it->j; 
            }
            it++;
        }

        // change the list of the better cell
        openList.remove(betterCell);
        inClosedList[betterCell.i][betterCell.j] = true;

        // checking the neighborhood of the cell
        for( i=-1 ; i<=1 ; i++)
            for( j=-1 ; j<=1 ; j++){
        		if(i==0 && j==0)
                            continue;

        		int cost = 10;                
        		if(!(i*i ^ j*j)){
                            cost = 14;
        		}	
                
                ipos_t neighbor(betterCell.i + i, betterCell.j + j);

                if(!isOccupatedPosition(neighbor) && !inClosedList[neighbor.i][neighbor.j]){
                    int G = costG[betterCell.i][betterCell.j] + cost,
                        H = 10*(abs(neighbor.i - iposEnd.i) + abs(neighbor.j - iposEnd.j)),
                        F = G + H;
                    
                    if(costF[neighbor.i][neighbor.j] == -1){
                        openList.push_back(neighbor);
                        costG[neighbor.i][neighbor.j] = G;
                        costH[neighbor.i][neighbor.j] = H;
                        costF[neighbor.i][neighbor.j] = F;
                        father[neighbor.i][neighbor.j] = betterCell;

                    }else if(G < costG[neighbor.i][neighbor.j]){
                            costG[neighbor.i][neighbor.j] = G;
                            costF[neighbor.i][neighbor.j] = F;
                            father[neighbor.i][neighbor.j] = betterCell;
                    }
                }
            }

    // checking exit condition
    }while( !openList.empty() && (!inClosedList[iposEnd.i][iposEnd.j]) );


    // create the return list
    if(inClosedList[iposEnd.i][iposEnd.j]){
        path.push_front(iposEnd);

        while(path.front() != iposBegin)
            path.push_front(father[path.front().i][path.front().j]);
    }

    return path;
}

// dpos_t class

dpos_t::dpos_t(){
    this->x = 0.0;
    this->x = 0.0;
}

dpos_t::dpos_t(double x, double y){
    this->x = x;
    this->x = y;
}

dpos_t::~dpos_t(){
}

dpos_t& dpos_t::operator =(const dpos_t &dpos){
    if (this != &dpos){
        x = dpos.x;
        y = dpos.y;
    }
    return *this;
}

bool dpos_t::operator ==(const dpos_t &dpos) const{
    return ((x==dpos.x) && (y==dpos.y));
}

bool dpos_t::operator !=(const dpos_t &dpos) const{
    return ((x!=dpos.x) || (y!=dpos.y));
}


// ipos_t class

ipos_t::ipos_t(){
    this->i = 0;
    this->j = 0;
}

ipos_t::ipos_t(int i, int j){
    this->i = i;
    this->j = j;
}

ipos_t::~ipos_t(){
}

ipos_t& ipos_t::operator =(const ipos_t &ipos){
    if (this != &ipos){
        i = ipos.i;
        j = ipos.j;
    }
    return *this;
}

bool ipos_t::operator ==(const ipos_t &ipos) const{
    return ((i==ipos.i) && (j==ipos.j));
}

bool ipos_t::operator !=(const ipos_t &ipos) const{
    return ((i!=ipos.i) || (j!=ipos.j));
}
