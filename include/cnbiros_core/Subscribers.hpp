#ifndef CNBIROS_CORE_SETSUBSCRIBERS_HPP
#define CNBIROS_CORE_SETSUBSCRIBERS_HPP

#include <ros/ros.h>
#include <unordered_map>

#include "cnbiros_core/Flags.hpp"

namespace cnbiros {
	namespace core {

typedef std::unordered_map<std::string, ros::Subscriber> MapSub;
typedef std::unordered_map<std::string, ros::Subscriber>::iterator MapSubIt;
typedef std::unordered_map<std::string, ros::Subscriber>::const_iterator MapSubConstIt;

class Subscribers {

	public:
		Subscribers(ros::NodeHandle* node);
		virtual ~Subscribers(void);

		MapSubIt Find(const std::string& topic);
		bool Exist(const std::string& topic);
		bool Remove(const std::string& topic);
		void Erase(void);
		
		bool Get(const std::string& topic, ros::Subscriber*& sub);

		template<class M>
		bool Add(const std::string& topic, void(*fp)(M));

		template<class M, class T>
		bool Add(const std::string& topic, void(T::*fp)(M), T* obj);
		
		template<class M, class T, class A>
		bool Add(const std::string& topic, void(T::*fp)(M), T* obj, A arg);

		template<class M>
		bool Add(const std::string& topic, 
				 const boost::function< void(const boost::shared_ptr<M const>&)> &callback);
		
	
	private:
		ros::NodeHandle* 	rosnode_;
		MapSub				rossubs_;


};

template<class M>
bool Subscribers::Add(const std::string& topic, void(*fp)(M)) {
	bool retcod = true;
	this->rossubs_[topic] = this->rosnode_->subscribe<M>(topic, CNBIROS_CORE_BUFFER_MESSAGES, fp);
	return retcod;
}

template<class M, class T>
bool Subscribers::Add(const std::string& topic, void(T::*fp)(M), T* obj) {
	bool retcod = true;
	this->rossubs_[topic] = this->rosnode_->subscribe<M>(topic, CNBIROS_CORE_BUFFER_MESSAGES, fp, obj);
	return retcod;
}

template<class M, class T, class A>
bool Subscribers::Add(const std::string& topic, void(T::*fp)(M), T* obj, A arg) {
	bool retcod = true;
	this->rossubs_[topic] = this->rosnode_->subscribe<M>(topic, CNBIROS_CORE_BUFFER_MESSAGES, 
							boost::bind<M>(T::fp, obj, _1, arg));
	return retcod;
}

template<class M>
bool Subscribers::Add(const std::string& topic, 
						 const boost::function< void(const boost::shared_ptr<M const>&)> &callback) {
	bool retcod = true;
	this->rossubs_[topic] = this->rosnode_->subscribe<M>(topic, CNBIROS_CORE_BUFFER_MESSAGES, callback);
	return retcod;
}



	}
}
	
#endif
