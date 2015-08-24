#ifndef OGMAP_H
#define	OGMAP_H
#include <list>
#include <vector>

class dpos_t {
public:
    dpos_t();
    dpos_t(double x, double y);
    virtual ~dpos_t();

    dpos_t& operator=(const dpos_t &dpos);
    bool operator == (const dpos_t &dpos) const;
    bool operator != (const dpos_t &dpos) const;

    double x;
    double y;
};

class ipos_t {
public:
    ipos_t();
    ipos_t(int i, int j);    
    virtual ~ipos_t();

    ipos_t& operator=(const ipos_t &ipos);
    bool operator == (const ipos_t &ipos) const;
    bool operator != (const ipos_t &ipos) const;

    int i;
    int j;
};


class OGMap {

public:
    #define EXCP_IMP 1

    OGMap(std::vector<float> dmap, const int width, const double p, const double s);

    virtual ~OGMap();

    //Get the horizontal size of the grid
    int getSizeX();

    //Get the vertical size of the grid
    int getSizeY();

    //Get the map scale
    double getScale();

    //Set the map scale
    void setScale(const double s);

    //Get the proportion (in pixels) between the image and the grid
    int getProportion();

    //Set the proportion (in pixels) between the image and the grid
    void setProportion(const int p);

    //Get the map horizontal offset
    double getOffsetX();

    //Set the map horizontal offset
    void setOffsetX(const double ox);

    //Get the map vertical offset
    double getOffsetY();

    // Set the map vertical offset
    void setOffsetY(const double oy);

    std::vector< std::vector<bool> > getMap();

    //Converts an index position to a distance position
    dpos_t itod(const ipos_t ipos);

    //Converts a distance position to an index position.
    ipos_t dtoi(const dpos_t dpos);

    //Check a position occupation.
    bool isOccupatedPosition(const ipos_t ipos) throw(int);
    bool isOccupatedPosition(const int i, const int j) throw(int);

    //Check occupation in the neighborhood
    bool neighborhood8(const ipos_t ipos);
    bool neighborhood16(const ipos_t ipos);
    bool neighborhood32(const ipos_t ipos);
    bool neighborhood64(const ipos_t ipos);

    //Set the occupation of a position.
    void setPositionOccupation(ipos_t ipos, const bool occupation) throw(int);

    //Print the map in ascii format.
    void printOccupationGrid(const char* fileName);

    //Use the A* algorithm to find the best free path between two points of the map.
    std::list<ipos_t> getPath(ipos_t iposBegin, ipos_t iposEnd) throw(int);

private:
    
    double scale; // Map's scale
    int proportion; //Map's proportion, i.e., number of pixels per cell
    std::vector< std::vector<bool> >map;//Occupation Map

    double offsetX;
    double offsetY;

};

#endif

