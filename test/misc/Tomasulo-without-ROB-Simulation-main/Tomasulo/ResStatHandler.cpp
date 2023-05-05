#include "ResStatHandler.h"

ResStatHandler::ResStatHandler(){
        LoadNum = 2;
        StoreNum = 2;
        AddNum = 2;
        DivNum = 1;
        JalNum = 1;
        BeqNum = 1;
    }
    
void ResStatHandler::incrementLoad(){
        LoadNum++;
    }
    void ResStatHandler::incrementStore(){
        StoreNum++;
    }
    void ResStatHandler::incrementAdd(){
        AddNum++;
    }
    void ResStatHandler::incrementDiv(){
        DivNum++;
    }
    void ResStatHandler::incrementJal(){
        JalNum++;
    }
    void ResStatHandler::incrementBeq(){
        BeqNum++;
    }


    void ResStatHandler::decrementLoad(){
        LoadNum--;
    }
    void ResStatHandler::decrementStore(){
        StoreNum--;
    }
    void ResStatHandler::decrementAdd(){
        AddNum--;
    }
    void ResStatHandler::decrementDiv(){
        DivNum--;
    }
    void ResStatHandler::decrementJal(){
        JalNum--;
    }
    void ResStatHandler::decrementBeq(){
        BeqNum--;
    }

    bool ResStatHandler::is_load_available(){
        return (LoadNum > 0);
    }

    bool ResStatHandler::is_store_available(){
        return (StoreNum > 0);
    }

    bool ResStatHandler::is_add_available(){
        return (AddNum > 0);
    }

    bool ResStatHandler::is_beq_available(){
        return (BeqNum > 0);
    }

    bool ResStatHandler::is_div_available(){
        return (DivNum > 0);
    }

    bool ResStatHandler::is_jal_available(){
        return (JalNum > 0);
    }
