#ifndef LEG_TRACKER_BOUNDING_BOX_H
#define LEG_TRACKER_BOUNDING_BOX_H

class BoundingBox
{

private:
  unsigned int fst_leg_id;
  unsigned int snd_leg_id;
  unsigned int peopleId;
  double uncertainty;
  double x_lower_limit;
  double x_upper_limit;
  double y_lower_limit;
  double y_upper_limit;
  int withoutUpdate;
  
public:
  BoundingBox() = delete;

  BoundingBox(unsigned int fst_leg_id, unsigned int snd_leg_id, unsigned int peopleId, 
      double fst_leg_x, double fst_leg_y, double snd_leg_x, double snd_leg_y)
  {
    this->fst_leg_id = fst_leg_id;
    this->snd_leg_id = snd_leg_id;
    this->peopleId = peopleId;
    uncertainty = 0.2;
    
    update(fst_leg_x, snd_leg_x, fst_leg_y, snd_leg_y);
  }
  
  void update(double fst_leg_x, double fst_leg_y, double snd_leg_x, double snd_leg_y)
  {
    withoutUpdate = 0;
    
    x_lower_limit = std::min(fst_leg_x, snd_leg_x);
    x_lower_limit -= uncertainty;
    
    x_upper_limit = std::max(fst_leg_x, snd_leg_x);
    x_upper_limit += uncertainty;
    
    y_lower_limit = std::min(fst_leg_y, snd_leg_y);
    y_lower_limit -= uncertainty;
    
    y_upper_limit = std::max(fst_leg_y, snd_leg_y);
    y_upper_limit += uncertainty;
  }
  
  void incrementWithoutUpdate()
  {
    withoutUpdate++;
  }
  
  bool isWithoutUpdate()
  {
    return withoutUpdate * 0.05 > 5.0;
  }
  
  unsigned int getFstLegId()
  {
    return fst_leg_id;
  }
  
  unsigned int getSndLegId()
  {
    return snd_leg_id;
  }
  
  double getXLowerLimit()
  {
    return x_lower_limit;
  }
  
  double getYLowerLimit()
  {
    return y_lower_limit;
  }
  
  double getXUpperLimit()
  {
    return x_upper_limit;
  }
  
  double getYUpperLimit()
  {
    return y_upper_limit;
  }

  int getPeopleId()
  {
    return peopleId;
  }

  void setPeopleId(int id)
  {
    peopleId = id;
  }
};

#endif
