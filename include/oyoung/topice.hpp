#ifndef OYOUNG_TOPICE_H
#define OYOUNG_TOPICE_H



namespace oyoung {
  
  template<typename Node>
  struct topice {
    
    topice(Node& node): _node(node) {}

    

  private:
    Node& _node;
  }
}

#endif /* OYOUNF_TOPICE_H */