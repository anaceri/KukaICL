#include <ICLQt/GUI.h>
#include <ICLMath/LinearTransform1D.h>
#include <ICLMath/StraightLine2D.h>
#include <ICLUtils/VisualizationDescription.h>

#include <vector>

namespace icl{
  class VisTool : public Configurable{
    GUI gui;
    GUI props;
    
    int W,H;
    std::vector<Point32f> trace;
    utils::Mutex visualizationsMutex;
    std::map<std::string,utils::VisualizationDescription> visualizations;
    
    struct TTool{
      LinearTransform1D tx, ty;
      inline Point32f operator()(const Point32f &p) const{
        return Point32f(tx(p.x), ty(p.y));
      }
    };
    
    public: 
    VisTool();
    
    QWidget *getRootWidget();
    void propertyChanged(const Property &p);
    
    void setCurrentPos(const Point32f &p);
    void setStartPoint(const Point32f &p);
    void setEndPoint(const Point32f &p);
    void setTraceVisible(bool on);
    
    void clearTrace();

    void addCustomVisualization(const utils::VisualizationDescription &vis, const std::string &id="default");
    void removeCustomVisualization(const std::string &id);
    void removeAllCustomVisualizations();

    private:
    
    TTool getTTool() const;
    void drawCircle2(DrawHandle &draw, const TTool &t, const Point32f &p, const std::string &id);
    void drawCircle(DrawHandle &draw, const TTool &t, const std::string &id);
    void drawLine(DrawHandle &draw, const TTool &t, const Point32f &a, 
                  const Point32f &b, const std::string &id);
    void drawQuadrangle(DrawHandle &draw, const TTool &t, const Point32f ps[4], 
                        const std::string &id);
    Point32f cvt(const StraightLine2D::Pos &p);
    void update();
  };
}

