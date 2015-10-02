/* Example implementation of a controller 
 *
 * NOTE: all methods are defined in the .cpp here but is in the real controller you also want a header of course
 */

#include <cstdlib>
#include <mutex>
#include <thread>

#include "bjos/bjos.h"
#include "bjos/controller/controller.h"

//WARNING: do not store pointers or STL objects here if not absolutely necessary
//NOTE: if you need to store a pointer use boost::offset_ptr (note that arrays can also be pointers...)
//NOTE: if you need to store a STL structure here use the boost::interprocess variants
struct SharedExampleData{
    int int_data;
    //add all other members here
};

class ExampleController : public Controller{
public:
    /* Initialize the class 
     * ALERT: dont do anything here that need shared data, mutexes or should only be done in the main process
     * NOTE: normally you only want to set your unitialized values (like pointers) to zero here
     */
    ExampleController(): _data(0) {}

    /* IMPLEMENT YOUR OWN FUNCTIONS HERE 
     * WARNING: you can assume that these methods are only called after initialization
     */
    int getInt(){
        //WARNING: for anything that modifies or reads shared data the mutex needs to be locked first
        //NOTE: you can also do mutex->lock() and mutex->unlock manually if needed, but normally you want to use a lock_guard (this also works with exceptions and guarentees unlocking)
        std::lock_guard<BJOS::Mutex> lock(*mutex);
        
        //do something with the data
        return _data->int_data;
        
        //NOTE: the mutex is automatically unlocked when this function is left
    }
    void setInt(int int_data){
        //NOTE: example of manual lock and unlock (that you normally dont want to to do)
        mutex->lock();
        _data->int_data = int_data;
        mutex->unlock();
    }
    
    /* If necessary you can overload the isAvailable method of the superclass */
    bool isAvailable(){
        //ALERT: first call the super class method to make sure the general interface is available because this function should be safe to call also if not initialized
        bool chk = Controller::isAvailable();
        if(!chk) return false;
        
        //do your own check if this interface is available and ready to use (so it methods are actually doing something real)
        
        //NOTE: if you do nothing here you can delete this method and just use the overloaded version
        return true;
    }
    
    /* Finalize this controller */
    ~ExampleController(){
        if(isMainInstance()){
            //WARNING: in most cases you want to check that you really need to delete things, because we cannot be certain that this controller is registered!
            if(!Controller::isAvailable()) return;
            
            //do own destruction for the main instance here...
        }else{
            //WARNING: in most cases you want to check that you really need to delete things, because we cannot be certain that this controller is loaded!
            if(!Controller::isAvailable()) return;
            
            //do own destruction for all other instances here...
        }
        
        //ALERT: always call the finalize method in the super class at the end of the destructor (unless the controller is not available)
        //ALERT: pass the datatype of the shared example data
        Controller::finalize<SharedExampleData>();
    }

//NOTE: these members are private because they should not be called directly
private:
    /* Initialize the main instance */ 
    void init(BJOS *bjos){
        //ALERT: always call the controller init method at the start of this method and pass the name of the controller and a reference to the shared data struct
        bool ret = Controller::init(bjos, "example", _data);
        
        //check if properly initialized
        if(!ret){
            //if not then the controller is not usable
            //most probably you just want to print an error and quit...
            std::cout << "Cannot initialize controller " << getControllerName() << std::endl;
            std::exit(0);
        }
        
        //initialize the main process... (dont forget to lock the mutex if you access the shared memory)
        //NOTE: you maybe want to start a thread here if you want to sync things periodically (make sure that you dont hang in this function)
    }
    /* load node is called for all childeren */
    void load(BJOS *bjos){
        //ALERT: always call the controller load method at the start of this method and pass the name of the controller and a reference to the shared data struct
        Controller::load(bjos, "example", _data);
        
        //load anything for a normal node if necessary
    }
    
    SharedExampleData *_data;
};

void mainProcess(){
    //reset BJOS (ALERT: this should only be done in the loader, because this will delete all other registered controllers ...)
    BJOS::init();
    
    //most likely you want to install the signal handler to make sure that we can exit properly on SIGTERM etc.
    BJOS::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();
    
    //make an unitialized controller
    ExampleController *example = new ExampleController;
    
    //initialize the controller
    bjos->initController(example);
    
    //do things until we got a request to stop the program
    while(bjos->isRunning()){
        std::cout << example->getInt() << std::endl;
        sleep(1);
    }
    
    //wait for other clients to unload (the main process should unload the latest)
    while(!example->canFinalize()){
        std::cout << "waiting for clients..." << std::endl;
        sleep(1);
    }
    
    //unload the controller
    delete example;
    
    //finish BJOS (ALERT: this should only be called in the loader that should remain active until all registered controllers are deregistered ...)
    BJOS::finalize();
}

void otherProcess(){
    //most likely you want to install the signal handler to make sure that we can exit properly on SIGTERM etc.
    BJOS::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();

    //make an unitialized controller
    ExampleController example;
    
    //try to retrieve an instance of a controller
    //ALERT: you should make sure that the controller you try to load is indeed the specified type
    bool ret = bjos->getController("example", &example);
    if(!ret){
        //cannot get the controller (the main process of this controller is not running)
        std::cout << "failed to get controller" << std::endl;
        return;
    }
    
    //do things ...
    std::cout << "setting the example controller to 5" << std::endl;
    example.setInt(5);
    
    //NOTE: the controller is automatically unloaded because its destructor is called when going out of scope (and you can always directly finalize as normal instance)
}

int main(){
    //use threads here to simulate the two processes here
    std::thread mainThd(mainProcess);
    
    //wait two seconds to allow the main process to start...
    sleep(2);
    
    std::thread otherThd(otherProcess);
    
    //wait to both are finished
    mainThd.join();
    otherThd.join();
}