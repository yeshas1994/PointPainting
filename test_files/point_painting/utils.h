#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>
#include <dirent.h>


namespace utils {

class file_iterator {
    public:
        file_iterator(std::string dir, std::string file_type);
        ~file_iterator();
        std::string get_next_file();
        std::shared_ptr<std::vector<std::string>> get_sorted_file_vector();
        std::shared_ptr<std::vector<std::string>> iterate();

    private:
        std::string dir_, file_type_;
        std::vector<std::string> filename_list_;
        bool iterated = false;

        DIR *opendir_;
};

file_iterator::file_iterator(const std::string dir, std::string file_type) {
    dir_ = dir;
    file_type_ = file_type;
    opendir_ = opendir(dir.c_str());
    if (opendir_  == nullptr) {
        std::printf("Error opening directory");
        return;
    }
}

file_iterator::~file_iterator(){
    closedir(opendir_);
};

std::string file_iterator::get_next_file(){
    std::string file_path = "done";

    bool valid_type = false;
    while(!valid_type){
        auto read_results = readdir(opendir_);
        if(read_results == NULL){
            // std::printf("Done getting path\n");
            return "done";
        }
        file_path = read_results->d_name;
        if(file_path.find(file_type_)!=std::string::npos){
            valid_type = true;
            filename_list_.push_back(file_path.insert(0,dir_));
        };
    }

    return file_path;
}

std::shared_ptr<std::vector<std::string>> file_iterator::get_sorted_file_vector(){
    if (file_type_ == ".png") {
        std::sort(filename_list_.begin(),filename_list_.end(),
                    [this](const std::string &string_a, const std::string &string_b){ // For Natural Sort
                        std::string string_a_l = string_a;
                        std::string string_b_l = string_b;
                        int directory_length = dir_.size();
                        string_a_l.erase(0,directory_length);
                        string_b_l.erase(0,directory_length);
                        int trim_pose = string_a_l.find("_");
                        string_a_l.erase(0,trim_pose+1);
                        string_b_l.erase(0,trim_pose+1);
                        string_a_l.erase(string_a_l.size()-file_type_.size(),file_type_.size());
                        string_b_l.erase(string_b_l.size()-file_type_.size(),file_type_.size());
                        return std::stoi(string_a_l)< std::stoi(string_b_l);
                    });
        return std::make_shared<std::vector<std::string>>(filename_list_);
    } else {
        std::sort(filename_list_.begin(),filename_list_.end(),
                    [this](const std::string &string_a, const std::string &string_b){ // For Natural Sort
                        std::string string_a_l = string_a;
                        std::string string_b_l = string_b;
                        int directory_length = dir_.size();
                        string_a_l.erase(0,directory_length);
                        string_b_l.erase(0,directory_length);
                        string_a_l.erase(0, 8);
                        string_b_l.erase(0, 8);
                        string_a_l.erase(string_a_l.size()-file_type_.size(),file_type_.size());
                        string_b_l.erase(string_b_l.size()-file_type_.size(),file_type_.size());
                        return std::stoi(string_a_l) < std::stoi(string_b_l);
                    });
        return std::make_shared<std::vector<std::string>>(filename_list_);
    }
}

std::shared_ptr<std::vector<std::string>> file_iterator::iterate(){
    if(!iterated){
        bool valid = true;
        while(valid){
            std::string path_name= get_next_file();
            if(path_name == "done"){
                valid = false;
                // std::printf("END\n");
            } else{
                // std::cout<<path_name<<std::endl;
            }
            iterated = true;
        }
    }else{
        // std::printf("Iterated Before. Quitting...\n");
    }
    return get_sorted_file_vector();
}

} // Namespace Utils