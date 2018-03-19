#ifndef LP_MP_FACTORS_MESSAGES_HXX
#define LP_MP_FACTORS_MESSAGES_HXX

#include <tuple>
#include <array>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <functional>
#include <utility>
#include <memory>
#include <limits>
#include <exception>
#include <typeinfo>
#include <type_traits>
#include <assert.h>
#include <cxxabi.h>

#include <mutex>

#include "template_utilities.hxx"
#include "function_existence.hxx"
#include "meta/meta.hpp"
#include "static_if.hxx"
#include "MemoryPool.h"

#include "memory_allocator.hxx"

#include "LP_MP.h"

// do zrobienia: remove these
//#include "factors/reparametrization_storage.hxx"  // also delete file
//#include "messages/message_storage.hxx"
#include <fstream>
#include <sstream>
#include <vector>
#include "vector"

#include "DD_ILP.hxx"

// this file provides message and factor containers. The factors and messages are plugged into the container and then every method call is dispatched correctly with static polymorphism and template tricks.

// do zrobienia: Introduce MessageConstraint and FactorConstraint for templates
// cleanup name inconsistencies: MessageType, MessageDispatcher etc

namespace LP_MP {

// we must check existence of functions in message classes. The necessary test code is concentrated here. 
namespace FunctionExistence {

// Macros to construct help functions for checking existence of member functions of classes
LP_MP_FUNCTION_EXISTENCE_CLASS(HasReceiveMessageFromRight,ReceiveMessageFromRight)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasReceiveMessageFromLeft, ReceiveMessageFromLeft)
   
LP_MP_FUNCTION_EXISTENCE_CLASS(HasReceiveRestrictedMessageFromRight,ReceiveRestrictedMessageFromRight)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasReceiveRestrictedMessageFromLeft, ReceiveRestrictedMessageFromLeft)

LP_MP_FUNCTION_EXISTENCE_CLASS(HasSendMessagesToRight,SendMessagesToRight)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasSendMessagesToLeft, SendMessagesToLeft)

LP_MP_FUNCTION_EXISTENCE_CLASS(HasRepamRight, RepamRight)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasRepamLeft, RepamLeft)

LP_MP_FUNCTION_EXISTENCE_CLASS(HasComputeLeftFromRightPrimal, ComputeLeftFromRightPrimal)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasComputeRightFromLeftPrimal, ComputeRightFromLeftPrimal)

LP_MP_FUNCTION_EXISTENCE_CLASS(HasCheckPrimalConsistency, CheckPrimalConsistency)

LP_MP_FUNCTION_EXISTENCE_CLASS(HasPropagatePrimal, PropagatePrimal)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasMaximizePotential, MaximizePotential)
LP_MP_FUNCTION_EXISTENCE_CLASS(HasMaximizePotentialAndComputePrimal, MaximizePotentialAndComputePrimal)

LP_MP_FUNCTION_EXISTENCE_CLASS(has_apply, apply)

LP_MP_FUNCTION_EXISTENCE_CLASS(has_create_constraints, create_constraints)

LP_MP_ASSIGNMENT_FUNCTION_EXISTENCE_CLASS(IsAssignable, operator[])
}

// function getters for statically dispatching ReceiveMessage and SendMessage to left and right side correctly, used in FactorContainer
template<typename MSG_CONTAINER>
struct LeftMessageFuncGetter
{
   using ConnectedFactorType = typename MSG_CONTAINER::RightFactorContainer;

   //constexpr static decltype(&MSG_CONTAINER::GetLeftMessage) GetMessageFunc() { return &MSG_CONTAINER::GetLeftMessage; }

   constexpr static decltype(&MSG_CONTAINER::ReceiveMessageFromRightContainer) GetReceiveFunc() { return &MSG_CONTAINER::ReceiveMessageFromRightContainer; }
   constexpr static decltype(&MSG_CONTAINER::ReceiveRestrictedMessageFromRightContainer) GetReceiveRestrictedFunc() { return &MSG_CONTAINER::ReceiveRestrictedMessageFromRightContainer; }
   constexpr static decltype(&MSG_CONTAINER::SendMessageToRightContainer) GetSendFunc() { return &MSG_CONTAINER::SendMessageToRightContainer; }
   template<typename LEFT_FACTOR, typename MSG_ITERATOR>
   constexpr static decltype(&MSG_CONTAINER::template SendMessagesToRightContainer<LEFT_FACTOR, MSG_ITERATOR>) GetSendMessagesFunc() 
   { return &MSG_CONTAINER::template SendMessagesToRightContainer<LEFT_FACTOR, MSG_ITERATOR>; }

#ifdef LP_MP_PARALLEL
   constexpr static decltype(&MSG_CONTAINER::ReceiveMessageFromRightContainerSynchronized) GetReceiveSynchronizedFunc() { return &MSG_CONTAINER::ReceiveMessageFromRightContainerSynchronized; }
   constexpr static decltype(&MSG_CONTAINER::SendMessageToRightContainerSynchronized) GetSendSynchronizedFunc() { return &MSG_CONTAINER::SendMessageToRightContainerSynchronized; }
   template<typename LEFT_FACTOR, typename MSG_ARRAY, typename ITERATOR>
   constexpr static decltype(&MSG_CONTAINER::template SendMessagesToRightContainerSynchronized<LEFT_FACTOR, MSG_ARRAY, ITERATOR>) GetSendMessagesSynchronizedFunc() 
   { return &MSG_CONTAINER::template SendMessagesToRightContainerSynchronized<LEFT_FACTOR, MSG_ARRAY, ITERATOR>; }
#endif

   constexpr static bool 
   ReceivesMessage()
   { return MSG_CONTAINER::receives_message_from_right_constexpr(); }

   constexpr static bool SendsMessage() 
   { return MSG_CONTAINER::sends_message_to_right_constexpr(); }

   constexpr static bool
   CanCallReceiveRestrictedMessage()
   { return MSG_CONTAINER::CanCallReceiveRestrictedMessageFromRightContainer(); }

   constexpr static bool 
   CanCallSendMessages()
   { return MSG_CONTAINER::CanCallSendMessagesToRightContainer(); }

   // do zrobienia: rename CanPropagatePrimalThroughMessage
   constexpr static bool CanComputePrimalThroughMessage()
   { return MSG_CONTAINER::CanComputeRightFromLeftPrimal(); }
   constexpr static decltype(&MSG_CONTAINER::ComputeRightFromLeftPrimal) GetComputePrimalThroughMessageFunc()
   { return &MSG_CONTAINER::ComputeRightFromLeftPrimal; }

   constexpr static Chirality GetChirality() { return Chirality::left; }
   constexpr static bool factor_holds_messages() { return MSG_CONTAINER::left_factor_holds_messages(); }
};

template<typename MSG_CONTAINER>
struct RightMessageFuncGetter
{
   using ConnectedFactorType = typename MSG_CONTAINER::LeftFactorContainer;

   //constexpr static decltype(&MSG_CONTAINER::GetRightMessage) GetMessageFunc() { return &MSG_CONTAINER::GetRightMessage; }

   constexpr static decltype(&MSG_CONTAINER::ReceiveMessageFromLeftContainer) GetReceiveFunc() { return &MSG_CONTAINER::ReceiveMessageFromLeftContainer; }
   constexpr static decltype(&MSG_CONTAINER::ReceiveRestrictedMessageFromLeftContainer) GetReceiveRestrictedFunc() { return &MSG_CONTAINER::ReceiveRestrictedMessageFromLeftContainer; }
   constexpr static decltype(&MSG_CONTAINER::SendMessageToLeftContainer) GetSendFunc() { return &MSG_CONTAINER::SendMessageToLeftContainer; }
   template<typename RIGHT_FACTOR, typename MSG_ITERATOR>
   constexpr static decltype(&MSG_CONTAINER::template SendMessagesToLeftContainer<RIGHT_FACTOR, MSG_ITERATOR>) GetSendMessagesFunc() 
   { return &MSG_CONTAINER::template SendMessagesToLeftContainer<RIGHT_FACTOR, MSG_ITERATOR>; }

#ifdef LP_MP_PARALLEL
   constexpr static decltype(&MSG_CONTAINER::ReceiveMessageFromLeftContainerSynchronized) GetReceiveSynchronizedFunc() { return &MSG_CONTAINER::ReceiveMessageFromLeftContainerSynchronized; }
   constexpr static decltype(&MSG_CONTAINER::SendMessageToLeftContainerSynchronized) GetSendSynchronizedFunc() { return &MSG_CONTAINER::SendMessageToLeftContainerSynchronized; }
   template<typename RIGHT_FACTOR, typename MSG_ARRAY, typename ITERATOR>
   constexpr static decltype(&MSG_CONTAINER::template SendMessagesToLeftContainerSynchronized<RIGHT_FACTOR, MSG_ARRAY, ITERATOR>) GetSendMessagesSynchronizedFunc() 
   { return &MSG_CONTAINER::template SendMessagesToLeftContainerSynchronized<RIGHT_FACTOR, MSG_ARRAY, ITERATOR>; }
#endif

   constexpr static bool ReceivesMessage()
   { return MSG_CONTAINER::receives_message_from_left_constexpr(); }

   constexpr static bool SendsMessage() 
   { return MSG_CONTAINER::sends_message_to_left_constexpr(); }

   constexpr static bool CanCallReceiveRestrictedMessage() 
   { return MSG_CONTAINER::CanCallReceiveRestrictedMessageFromLeftContainer(); }

   constexpr static bool
   CanCallSendMessages()
   { return MSG_CONTAINER::CanCallSendMessagesToLeftContainer(); }

   constexpr static bool CanComputePrimalThroughMessage()
   { return MSG_CONTAINER::CanComputeLeftFromRightPrimal(); }
   constexpr static decltype(&MSG_CONTAINER::ComputeLeftFromRightPrimal) GetComputePrimalThroughMessageFunc()
   { return &MSG_CONTAINER::ComputeLeftFromRightPrimal; }

   constexpr static Chirality GetChirality() { return Chirality::right; }
   constexpr static bool factor_holds_messages() { return MSG_CONTAINER::right_factor_holds_messages(); }
};

template<class MSG_CONTAINER, template<typename> class FuncGetter>
struct MessageDispatcher
{
   using ConnectedFactorType = typename FuncGetter<MSG_CONTAINER>::ConnectedFactorType; // this is the type of factor container to which the message is connected

   static void ReceiveMessage(MSG_CONTAINER& t)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetReceiveFunc();
      (t.*staticMemberFunc)();
   }
#ifdef LP_MP_PARALLEL
   static void ReceiveMessageSynchronized(MSG_CONTAINER& t)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetReceiveSynchronizedFunc();
      (t.*staticMemberFunc)();
   }
#endif

   constexpr static bool CanCallReceiveRestrictedMessage() { return FuncGetter<MSG_CONTAINER>::CanCallReceiveRestrictedMessage(); }
   static void ReceiveRestrictedMessage(MSG_CONTAINER& t)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetReceiveRestrictedFunc();
      (t.*staticMemberFunc)();
   }

   constexpr static bool SendsMessage() { return FuncGetter<MSG_CONTAINER>::SendsMessage(); }
   constexpr static bool ReceivesMessage() { return FuncGetter<MSG_CONTAINER>::ReceivesMessage(); }

   template<typename FACTOR_TYPE>
   static void SendMessage(FACTOR_TYPE* f, MSG_CONTAINER& t, const REAL omega)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetSendFunc();
      (t.*staticMemberFunc)(f, omega);
   }

#ifdef LP_MP_PARALLEL
   template<typename FACTOR_TYPE>
   static void SendMessageSynchronized(FACTOR_TYPE* f, MSG_CONTAINER& t, const REAL omega)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetSendSynchronizedFunc();
      (t.*staticMemberFunc)(f, omega);
   }
#endif

   // batch message sending
   constexpr static bool CanCallSendMessages() { return FuncGetter<MSG_CONTAINER>::CanCallSendMessages(); }

   template<typename FACTOR, typename MSG_ITERATOR>
   static void SendMessages(const FACTOR& f, MSG_ITERATOR msgs_begin, MSG_ITERATOR msgs_end, const REAL omega)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::template GetSendMessagesFunc<FACTOR, MSG_ITERATOR>();
      (*staticMemberFunc)(f, msgs_begin, msgs_end, omega);
   }

#ifdef LP_MP_PARALLEL
   template<typename FACTOR, typename MSG_ARRAY, typename ITERATOR>
   static void SendMessagesSynchronized(const FACTOR& f, const MSG_ARRAY& msgs, ITERATOR omegaBegin)
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::template GetSendMessagesSynchronizedFunc<FACTOR, MSG_ARRAY, ITERATOR>();
      (*staticMemberFunc)(f, msgs, omegaBegin);
   }
#endif

   constexpr static bool CanComputePrimalThroughMessage() // do zrobienia: return false, if the factor from which this is called computes its own primal already
   {
      return FuncGetter<MSG_CONTAINER>::CanComputePrimalThroughMessage();
   }

   static void ComputePrimalThroughMessage(MSG_CONTAINER& t) 
   {
      auto staticMemberFunc = FuncGetter<MSG_CONTAINER>::GetComputePrimalThroughMessageFunc();
      return (t.*staticMemberFunc)();
   }
   constexpr static Chirality GetChirality() { return FuncGetter<MSG_CONTAINER>::GetChirality(); }
   constexpr static bool factor_holds_messages() { return FuncGetter<MSG_CONTAINER>::factor_holds_messages(); }
};

template<INDEX NO_ELEMENTS, typename T>
class FixedSizeMessageContainer : public std::array<T*,NO_ELEMENTS> {
public: 
   FixedSizeMessageContainer() { this->fill(nullptr); }
   ~FixedSizeMessageContainer() {}
   void push_back(T* t) {
      // do zrobienia: possibly use binary search when NO_ELEMENTS is bigger than some threshold
      for(INDEX i=0; i<NO_ELEMENTS; ++i) {
         if((*this)[i] == nullptr) {
            (*this)[i] = t;
            return;
         }
      }
      throw std::range_error("added more messages than can be held");
   }
   constexpr INDEX size() const { return NO_ELEMENTS; }
};

// actually hold messages here
template<INDEX NO_ELEMENTS, typename MSG_CONTAINER_TYPE>
class FixedSizeMessageContainer2 : public std::array<char, sizeof(MSG_CONTAINER_TYPE) * NO_ELEMENTS>
{
public:
	using msg_type = typename MSG_CONTAINER_TYPE::MessageType;
	static constexpr INDEX size_in_bytes = sizeof(MSG_CONTAINER_TYPE) * NO_ELEMENTS;
	FixedSizeMessageContainer2() 
	{
		std::fill((char*) this, (char*) this + size_in_bytes, 0);
	}
	~FixedSizeMessageContainer2() 
	{
		static_assert(sizeof(*this) == sizeof(MSG_CONTAINER_TYPE) * NO_ELEMENTS);
	}
	template<typename LEFT_FACTOR, typename RIGHT_FACTOR>
	void push_back(LEFT_FACTOR* l, RIGHT_FACTOR* r, msg_type msg)
	{
		// determine at which place to insert: iterate over memory in strides of size of MSG_CONTAINER and check whether it is zero
		for(INDEX pos = 0; pos<size(); ++pos) {
			char* ptr = (char*) this + pos*sizeof(MSG_CONTAINER_TYPE);
			auto* ptr_end = std::find_if(ptr, ptr+sizeof(MSG_CONTAINER_TYPE), [](char x) { return x!=0; });
			if(ptr_end == ptr+sizeof(MSG_CONTAINER_TYPE)) { 
				MSG_CONTAINER_TYPE* msg = new(ptr) MSG_CONTAINER_TYPE(l,r,msg); 
			}
		}
		assert(false); // already holds maximum number of messages
	} 

	constexpr INDEX size() const { return NO_ELEMENTS; }
};

// holds at most NO_ELEMENTS in std::array. Unused entries have nullptr in them
template<INDEX NO_ELEMENTS, typename T>
class UpToFixedSizeMessageContainer : public std::array<T*,NO_ELEMENTS> {
public:
   UpToFixedSizeMessageContainer() : size_(0) { this->fill(nullptr); }
   ~UpToFixedSizeMessageContainer() { 
      static_assert(NO_ELEMENTS > 0, "");
   }
   void push_back(T* t) {
      assert(size_ < NO_ELEMENTS);
      (*this)[size_] = t;
      ++size_;
   }
   INDEX size() const { return size_; }
   auto end() const -> decltype(this->end()) { return this->begin() + size(); }


private:
   unsigned char size_;
};

// for one element we do not need to store its size explicitly
template<typename T>
class UpToFixedSizeMessageContainer<1,T> : public std::array<T*,1> {
public:
   UpToFixedSizeMessageContainer() { this->fill(nullptr); }
   void push_back(T* t) {
      assert((*this)[0] == nullptr);
      (*this)[0] = t;
   }
   INDEX size() const { return (*this)[0] == nullptr ? 0 : 1; } 
   auto end() const -> decltype(this->end()) { return this->begin() + size(); }
};

template<typename T>
class UpToFixedSizeMessageContainer<2,T> : public std::array<T*,2> {
public:
   UpToFixedSizeMessageContainer() { this->fill(nullptr); }
   void push_back(T* t) {
      if((*this)[0] == nullptr) {
         (*this)[0] = t; 
      } else if((*this)[1] == nullptr) {
         (*this)[1] = t; 
      } else {
         assert(false);
      }
   }
   INDEX size() const {
      return ((*this)[0] != nullptr)*1 + ((*this)[1] != nullptr)*1;
      //if((*this)[0] == nullptr) {
      //   (*this)[0] = t; 
      //} else if((*this)[1] == nullptr) {
      //   (*this)[1] = t; 
      //} else {
      //return 2; }
      //} 
}
   auto end() const -> decltype(this->end()) { return this->begin() + size(); }
};

template<typename MSG_CONTAINER, bool HOLD>
struct next_left_message_container {
   MSG_CONTAINER* next_msg() const { assert(false); return nullptr; }
   void next_msg(MSG_CONTAINER*) { assert(false); }
};

template<typename MSG_CONTAINER>
struct next_left_message_container<MSG_CONTAINER,true> {
   void next_msg(MSG_CONTAINER* m) { next = m; }
   MSG_CONTAINER* next_msg() const { return next; }
   MSG_CONTAINER* next = nullptr;
};
   
template<typename MSG_CONTAINER, bool HOLD>
struct next_right_message_container {
   MSG_CONTAINER* next_msg() const { assert(false); return nullptr; }
   void next_msg(MSG_CONTAINER*) { assert(false); }
};

template<typename MSG_CONTAINER>
struct next_right_message_container<MSG_CONTAINER,true> {
   void next_msg(MSG_CONTAINER* m) { next = m; }
   MSG_CONTAINER* next_msg() const { return next; }
   MSG_CONTAINER* next = nullptr;
};


template<typename M, Chirality CHIRALITY>
class VariableSizeMessageContainer
{
public:
   VariableSizeMessageContainer() : m_(nullptr), size_(0) {}
   INDEX size() const { return size_; }
   void push_back(M* m) { // actually it is push_front
      if(CHIRALITY == Chirality::right) {
         static_cast<typename M::next_right_message_container_type*>(m)->next_msg(m_);
      } else {
         static_cast<typename M::next_left_message_container_type*>(m)->next_msg(m_);
      }
      m_ = m;
      ++size_;
   }

   class iterator {
      public:
         iterator(M* m) : m_(m) {}
         iterator operator++() {
            if(CHIRALITY == Chirality::right) {
               //m_ = m_->next_right_message_container::next_msg();
               m_ = static_cast<typename M::next_right_message_container_type*>(m_)->next_msg();
            } else {
               //m_ = m_->next_left_message_container::next_msg();
               m_ = static_cast<typename M::next_left_message_container_type*>(m_)->next_msg();
            }
            return *this;
         }
         M* operator*() const { return m_; } 
         bool operator==(const iterator& o) const { return m_ == o.m_; }
         bool operator!=(const iterator& o) const { return m_ != o.m_; }
      private:
         M* m_;
   };

   iterator begin() const {
      return iterator(m_);
   }
   iterator end() const {
      return iterator(nullptr);
   }

private:
   M* m_;
   INDEX size_;
};

// N=0 means variable number of messages, > 0 means compile time fixed number of messages and <0 means at most compile time number of messages
// see config.hxx for shortcuts
template<SIGNED_INDEX N, typename MESSAGE_CONTAINER_TYPE, Chirality CHIRALITY>
struct MessageContainerSelector {
   using type = 
      typename std::conditional<(N > 0), FixedSizeMessageContainer<INDEX(N),MESSAGE_CONTAINER_TYPE>,
      typename std::conditional<(N < 0), UpToFixedSizeMessageContainer<INDEX(-N),MESSAGE_CONTAINER_TYPE>, 
                                         VariableSizeMessageContainer<MESSAGE_CONTAINER_TYPE,CHIRALITY> >::type >::type;
};

// Class holding message and left and right factor
// do zrobienia: possibly replace {LEFT|RIGHT}_FACTOR_NO by their type
template<typename MESSAGE_TYPE, 
         INDEX LEFT_FACTOR_NO, INDEX RIGHT_FACTOR_NO,
         message_passing_schedule MPS,
         SIGNED_INDEX NO_OF_LEFT_FACTORS, SIGNED_INDEX NO_OF_RIGHT_FACTORS,
         typename FACTOR_MESSAGE_TRAIT, 
         INDEX MESSAGE_NO
         >
class MessageContainer : //public MessageStorageSelector<MESSAGE_SIZE,true>::type, 
                           public MessageTypeAdapter
                         // when NO_OF_LEFT_FACTORS is zero, we hold factors in linked list
                         ,public next_left_message_container<MessageContainer<MESSAGE_TYPE,LEFT_FACTOR_NO,RIGHT_FACTOR_NO,MPS,NO_OF_LEFT_FACTORS,NO_OF_RIGHT_FACTORS,FACTOR_MESSAGE_TRAIT,MESSAGE_NO>,NO_OF_LEFT_FACTORS == 0> 
                         ,public next_right_message_container<MessageContainer<MESSAGE_TYPE,LEFT_FACTOR_NO,RIGHT_FACTOR_NO,MPS,NO_OF_LEFT_FACTORS,NO_OF_RIGHT_FACTORS,FACTOR_MESSAGE_TRAIT,MESSAGE_NO>,NO_OF_RIGHT_FACTORS == 0>
{
public:
   using leftFactorNumber_t = std::integral_constant<INDEX, LEFT_FACTOR_NO>;
   static constexpr INDEX leftFactorNumber = LEFT_FACTOR_NO;
   static constexpr INDEX rightFactorNumber = RIGHT_FACTOR_NO;

   using MessageContainerType = MessageContainer<MESSAGE_TYPE, LEFT_FACTOR_NO, RIGHT_FACTOR_NO, MPS, NO_OF_LEFT_FACTORS, NO_OF_RIGHT_FACTORS, FACTOR_MESSAGE_TRAIT, MESSAGE_NO>;
   using MessageType = MESSAGE_TYPE;
   using next_left_message_container_type = next_left_message_container<MessageContainerType,NO_OF_LEFT_FACTORS == 0>;
   using next_right_message_container_type = next_right_message_container<MessageContainerType,NO_OF_RIGHT_FACTORS == 0>;
   //typedef typename MessageStorageSelector<MESSAGE_SIZE,true>::type MessageStorageType; // do zrobienia: true is just for now. In general, message need not hold actual message, except when some factor is reparametrized implicitly

   // structures used in FactorContainer to hold pointers to messages
   using LeftMessageContainerStorageType = typename MessageContainerSelector<NO_OF_LEFT_FACTORS, MessageContainerType, Chirality::left>::type;
   using RightMessageContainerStorageType = typename MessageContainerSelector<NO_OF_RIGHT_FACTORS, MessageContainerType, Chirality::right>::type;

   // FactorContainer
   using LeftFactorContainer = meta::at_c<typename FACTOR_MESSAGE_TRAIT::FactorList, leftFactorNumber>;
   using RightFactorContainer = meta::at_c<typename FACTOR_MESSAGE_TRAIT::FactorList, rightFactorNumber>;
   // Factor
   using LeftFactorType = typename LeftFactorContainer::FactorType;
   using RightFactorType = typename RightFactorContainer::FactorType;

   constexpr static bool left_factor_holds_messages() { return NO_OF_LEFT_FACTORS != 0; }
   constexpr static bool right_factor_holds_messages() { return NO_OF_RIGHT_FACTORS != 0; }
   

   template<typename ...ARGS>
   MessageContainer(LeftFactorContainer* const l, RightFactorContainer* const r, ARGS... args) 
   : msg_op_(args...),
   leftFactor_(l),
   rightFactor_(r)
   {
      leftFactor_->template AddMessage<MessageDispatcher<MessageContainerType, LeftMessageFuncGetter>, MessageContainerType>(this);
      rightFactor_->template AddMessage<MessageDispatcher<MessageContainerType, RightMessageFuncGetter>, MessageContainerType>(this);
   }

   /* seems not to work, as arguments are matched greedily???
   template<typename ...ARGS>
   MessageContainer(ARGS... args, LeftFactorContainer* const l, RightFactorContainer* const r) 
   : msg_op_(args...),
   leftFactor_(l),
   rightFactor_(r)
   {
      leftFactor_->template AddMessage<MessageDispatcher<MessageContainerType, LeftMessageFuncGetter>, MessageContainerType>(this);
      rightFactor_->template AddMessage<MessageDispatcher<MessageContainerType, RightMessageFuncGetter>, MessageContainerType>(this);
   }
   */

   MessageContainer(MESSAGE_TYPE msg_op, LeftFactorContainer* const l, RightFactorContainer* const r) 
      ://MessageStorageType(),
      msg_op_(msg_op),
      leftFactor_(l),
      rightFactor_(r)
   {
      leftFactor_->template AddMessage<MessageDispatcher<MessageContainerType, LeftMessageFuncGetter>, MessageContainerType>(this);
      rightFactor_->template AddMessage<MessageDispatcher<MessageContainerType, RightMessageFuncGetter>, MessageContainerType>(this);
      //int status;
      //std::cout << "msg holding type = " << abi::__cxa_demangle(typeid(*this).name(),0,0,&status) << "\n";
      //std::cout << FunctionExistence::IsAssignable<RightFactorContainer,REAL,INDEX>() << "\n";
      //std::cout << "msg holding type = " << abi::__cxa_demangle(typeid(msg_op_).name(),0,0,&status) << "\n";
      //std::cout << "left factor number = " << leftFactorNumber << "\n";
      //std::cout << "right factor number = " << rightFactorNumber << "\n";
      //std::cout << "left factor type = " << abi::__cxa_demangle(typeid(LeftFactorContainer).name(),0,0,&status) << "\n";
      //std::cout << "right factor type = " << abi::__cxa_demangle(typeid(RightFactorContainer).name(),0,0,&status) << "\n";
      // register messages in factors
   }
   ~MessageContainer() {
      static_assert(meta::unique<typename FACTOR_MESSAGE_TRAIT::MessageList>::size() == FACTOR_MESSAGE_TRAIT::MessageList::size(), 
            "Message list must have unique elements");
      static_assert(MESSAGE_NO >= 0 && MESSAGE_NO < FACTOR_MESSAGE_TRAIT::MessageList::size(), "message number must be smaller than length of message list");
      static_assert(leftFactorNumber < FACTOR_MESSAGE_TRAIT::FactorList::size(), "left factor number out of bound");
      static_assert(rightFactorNumber < FACTOR_MESSAGE_TRAIT::FactorList::size(), "right factor number out of bound");
      // do zrobienia: put message constraint here, i.e. which methods MESSAGE_TYPE must minimally implement
   } 
   
   // overloaded new so that factor containers are allocated by global block allocator consecutively
   void* operator new(std::size_t size)
   {
      assert(size == sizeof(MessageContainerType));
      //INDEX s = size/sizeof(REAL);
      //if(size % sizeof(REAL) != 0) { s++; }
      //return (void*) global_real_block_allocator.allocate(s,1);
      return Allocator::get().allocate(1);
   }
   void operator delete(void* mem)
   {
      Allocator::get().deallocate((MessageContainerType*) mem);
      //assert(false);
      //global_real_block_allocator.deallocate(mem,sizeof(FactorContainerType));
   }

   virtual MessageTypeAdapter* clone(FactorTypeAdapter* l, FactorTypeAdapter* r) const final
   {
      auto* m = new MessageContainer(msg_op_, static_cast<LeftFactorContainer*>(l), static_cast<RightFactorContainer*>(r));
      return m; 
   }

   void send_message_to_left(const REAL omega = 1.0) 
   {
      send_message_to_left(rightFactor_->GetFactor(), omega);
   }
   void send_message_to_left(RightFactorType* r, const REAL omega)
   {
     msg_op_.send_message_to_left(*r, *static_cast<MessageContainerView<Chirality::right>*>(this), omega); 
   }

#ifdef LP_MP_PARALLEL
   void send_message_to_left_synchronized(const REAL omega = 1.0) 
   {
      send_message_to_left_synchronized(rightFactor_->GetFactor(), omega);
   }
   void send_message_to_left_synchronized(RightFactorType* r, const REAL omega)
   {
     auto& mtx = GetLeftFactor()->mutex_;
     std::unique_lock<std::recursive_mutex> lck(mtx,std::defer_lock);
     if(lck.try_lock()) {
       msg_op_.send_message_to_left(*r, *static_cast<MessageContainerView<Chirality::right>*>(this), omega); 
     } else {
#ifndef NDEBUG
       if(debug()) {
         std::cout << "locking failed for " << GetLeftFactor() << "\n";
       }
#endif
     }
   }
#endif

   void send_message_to_right(const REAL omega = 1.0) 
   {
      send_message_to_right(leftFactor_->GetFactor(), omega);
   }
   void send_message_to_right(LeftFactorType* l, const REAL omega)
   {
     msg_op_.send_message_to_right(*l, *static_cast<MessageContainerView<Chirality::left>*>(this), omega); 
   }

#ifdef LP_MP_PARALLEL
   void send_message_to_right_synchronized(const REAL omega = 1.0) 
   {
      send_message_to_right_synchronized(leftFactor_->GetFactor(), omega);
   }
   void send_message_to_right_synchronized(LeftFactorType* l, const REAL omega)
   {
     auto& mtx = GetRightFactor()->mutex_;
     std::unique_lock<std::recursive_mutex> lck(mtx,std::defer_lock);
     if(lck.try_lock()) {
       msg_op_.send_message_to_right(*l, *static_cast<MessageContainerView<Chirality::left>*>(this), omega); 
     } else {
#ifndef NDEBUG
       if(debug()) {
         std::cout << "locking failed for " << GetRightFactor() << "\n";
       }
#endif
     }
   }
#endif

   constexpr static bool
   CanCallReceiveMessageFromRightContainer()
   { 
      return MPS == message_passing_schedule::left || MPS==message_passing_schedule::full;
      // obsolete
      return FunctionExistence::HasReceiveMessageFromRight<MessageType, void, 
      RightFactorType, MessageContainerType>(); 
   }
   void ReceiveMessageFromRightContainer()
   {
#ifndef NDEBUG
      const REAL before_left_lb = leftFactor_->LowerBound();
      const REAL before_right_lb = rightFactor_->LowerBound();
#endif

      send_message_to_left();

#ifndef NDEBUG
      const REAL after_left_lb = leftFactor_->LowerBound();
      const REAL after_right_lb = rightFactor_->LowerBound();
      assert(before_left_lb + before_right_lb <= after_left_lb + after_right_lb + eps); 
#endif
   }

#ifdef LP_MP_PARALLEL
   void ReceiveMessageFromRightContainerSynchronized()
   {
      send_message_to_left_synchronized();
   }
#endif

   constexpr static bool
   CanCallReceiveRestrictedMessageFromRightContainer()
   { 
      return FunctionExistence::HasReceiveRestrictedMessageFromRight<MessageType, void, 
      RightFactorType, MessageContainerType>(); // do zrobienia: signature is slighly different: MessageContainerType is not actually used
   }
   void ReceiveRestrictedMessageFromRightContainer()
   {
      rightFactor_->conditionally_init_primal(leftFactor_->primal_access_);
      msg_op_.ReceiveRestrictedMessageFromRight(*(rightFactor_->GetFactor()), *static_cast<OneSideMessageContainerView<Chirality::left>*>(this));
   }

   constexpr static bool 
   CanCallReceiveMessageFromLeftContainer()
   { 
      return MPS == message_passing_schedule::right || MPS == message_passing_schedule::full;
   }
   void ReceiveMessageFromLeftContainer()
   { 
#ifndef NDEBUG
      const REAL before_left_lb = leftFactor_->LowerBound();
      const REAL before_right_lb = rightFactor_->LowerBound();
#endif

      send_message_to_right();

#ifndef NDEBUG
      const REAL after_left_lb = leftFactor_->LowerBound();
      const REAL after_right_lb = rightFactor_->LowerBound();
      assert(before_left_lb + before_right_lb <= after_left_lb + after_right_lb + eps); 
#endif
   }

#ifdef LP_MP_PARALLEL
   void ReceiveMessageFromLeftContainerSynchronized()
   { 
      send_message_to_right_synchronized();
   }
#endif

   constexpr static bool
   CanCallReceiveRestrictedMessageFromLeftContainer()
   { 
      return FunctionExistence::HasReceiveRestrictedMessageFromLeft<MessageType, void, 
      LeftFactorType, MessageContainerType>(); 
   }
   void ReceiveRestrictedMessageFromLeftContainer()
   {
      leftFactor_->conditionally_init_primal(rightFactor_->primal_access_);
      msg_op_.ReceiveRestrictedMessageFromLeft(*(leftFactor_->GetFactor()), *static_cast<OneSideMessageContainerView<Chirality::right>*>(this));
   }


   constexpr static bool 
   CanCallSendMessageToRightContainer()
   { 
      return MPS == message_passing_schedule::left || MPS == message_passing_schedule::full || MPS == message_passing_schedule::only_send;
   }

   void SendMessageToRightContainer(LeftFactorType* l, const REAL omega)
   {
      send_message_to_right(l, omega);
   }

#ifdef LP_MP_PARALLEL
   void SendMessageToRightContainerSynchronized(LeftFactorType* l, const REAL omega)
   {
      send_message_to_right_synchronized(l, omega);
   }
#endif

   constexpr static bool
   CanCallSendMessageToLeftContainer()
   { 
      return MPS == message_passing_schedule::right || MPS == message_passing_schedule::full || MPS == message_passing_schedule::only_send;
   }

   void SendMessageToLeftContainer(RightFactorType* r, const REAL omega)
   {
      send_message_to_left(r, omega);
   }

#ifdef LP_MP_PARALLEL
   void SendMessageToLeftContainerSynchronized(RightFactorType* r, const REAL omega)
   {
      send_message_to_left_synchronized(r, omega);
   }
#endif

   constexpr static bool CanCallSendMessagesToLeftContainer()
   {
      // possibly the below is to complicated. meta::find will be easier
      constexpr INDEX msg_array_number = RightFactorContainer::template FindMessageDispatcherTypeIndex<MessageDispatcher<MessageContainerType,RightMessageFuncGetter>>();
      using msg_container_type = meta::at_c<typename RightFactorContainer::msg_container_type_list, msg_array_number>;
      using MSG_ARRAY_ITERATOR = decltype(std::declval<msg_container_type>().begin());
      return FunctionExistence::HasSendMessagesToLeft<MessageType, void, RightFactorType, MSG_ARRAY_ITERATOR, MSG_ARRAY_ITERATOR, REAL>();
   }

   template<Chirality C> class MessageContainerView; // forward declaration. Put MessageIteratorView after definition of MessageContainerView

   template<Chirality CHIRALITY, typename MESSAGE_ITERATOR>
   struct MessageIteratorView {
     MessageIteratorView(MESSAGE_ITERATOR it) : it_(it) {} 
     MessageContainerView<CHIRALITY>& operator*() const {
       return *(static_cast<MessageContainerView<CHIRALITY>*>( *it_ )); 
     }
     MessageIteratorView<CHIRALITY,MESSAGE_ITERATOR>& operator++() {
       ++it_;
       return *this;
     }
     bool operator==(const MessageIteratorView<CHIRALITY,MESSAGE_ITERATOR>& o) const {
       return it_ == o.it_; 
     }
     bool operator!=(const MessageIteratorView<CHIRALITY,MESSAGE_ITERATOR>& o) const {
       return it_ != o.it_; 
     }
     private:
     MESSAGE_ITERATOR it_;
   };

#ifdef LP_MP_PARALLEL
   template<Chirality CHIRALITY, typename MESSAGE_ITERATOR, typename LOCK_ITERATOR>
   struct MessageIteratorViewSynchronized {
     MessageIteratorViewSynchronized(MESSAGE_ITERATOR it, LOCK_ITERATOR lock_it) : it_(it), lock_it_(lock_it) {}
     MessageContainerView<CHIRALITY>& operator*() const {
       return *(static_cast<MessageContainerView<CHIRALITY>*>( *it_ )); 
     }
     MessageIteratorViewSynchronized<CHIRALITY,MESSAGE_ITERATOR,LOCK_ITERATOR>& operator++() {
       ++it_;
       ++lock_it_;
       while(*lock_it_ == false) { // this will always terminate: the lock_rec has one more entry than there are msgs and last entry is always true
         ++it_;
         ++lock_it_; 
       }
       return *this;
     }
     bool operator==(const MessageIteratorViewSynchronized<CHIRALITY,MESSAGE_ITERATOR,LOCK_ITERATOR>& o) const {
       return it_ == o.it_; 
     }
     bool operator!=(const MessageIteratorViewSynchronized<CHIRALITY,MESSAGE_ITERATOR,LOCK_ITERATOR>& o) const {
       return it_ != o.it_; 
     }
     private:
     MESSAGE_ITERATOR it_;
     LOCK_ITERATOR lock_it_;
   };
#endif

   template<typename IT, typename LOCK_ITERATOR>
   struct omega_iterator_with_lock {
     omega_iterator_with_lock(IT it, LOCK_ITERATOR lock_it) : it_(it), lock_it_(lock_it) {}
     omega_iterator_with_lock(const omega_iterator_with_lock& o) : it_(o.it_), lock_it_(o.lock_it_) {}
     omega_iterator_with_lock& operator++() {
       ++it_;
       ++lock_it_;
       while(*lock_it_ == false) { // this will always terminate: the lock_it_ has one more entry than there are elements pointed to by it_ and last entry is always true
         ++it_;
         ++lock_it_;
       }
       return *this;
     }
     omega_iterator_with_lock operator+(const INDEX s) {
        omega_iterator_with_lock o(*this);
        for(INDEX i=0; i<s; ++i) {
           ++o;
        }
        return o;
     }
     auto operator*() const { return *it_; }
     bool operator==(const omega_iterator_with_lock<IT,LOCK_ITERATOR>& o) const { return it_ == o.it_; }
     bool operator!=(const omega_iterator_with_lock<IT,LOCK_ITERATOR>& o) const { return it_ != o.it_; }

     private:
     IT it_;
     LOCK_ITERATOR lock_it_;
   };

   // rename to send_messages_to_left_container
   template<typename RIGHT_FACTOR, typename MSG_ITERATOR>
   static void SendMessagesToLeftContainer(const RIGHT_FACTOR& rightFactor, MSG_ITERATOR msgs_begin, MSG_ITERATOR msgs_end, const REAL omega) 
   {
      using MessageIteratorType = MessageIteratorView<Chirality::right, MSG_ITERATOR>;
      return MessageType::SendMessagesToLeft(rightFactor, MessageIteratorType(msgs_begin), MessageIteratorType(msgs_end), omega);
   }

#ifdef LP_MP_PARALLEL
   template<typename RIGHT_FACTOR, typename MSG_ARRAY, typename ITERATOR, typename LOCK_VECTOR>
   static void SendMessagesToLeftContainerSynchronized_impl(const RIGHT_FACTOR& rightFactor, const MSG_ARRAY& msgs, ITERATOR omegaBegin, LOCK_VECTOR& lock_rec) 
   {
      // first lock as many adjacent factors as possible.
      auto lock_it = lock_rec.begin();
      for(auto it=msgs.begin(); it!=msgs.end(); ++it, ++lock_it) {
        auto& mtx = (*it)->GetLeftFactor()->mutex_;
        if(mtx.try_lock()) { // mark that factor was locked by this process
          *lock_it = true;
        } else {
          *lock_it = false; 
#ifndef NDEBUG
          if(debug()) {
            std::cout << "locking failed for " << (*it)->GetLeftFactor() << "\n";
          }
#endif
        }
      }
      assert(lock_it+1 == lock_rec.end());
      //std::fill(lock_rec.begin(), lock_rec.end(), true);

      using MessageIteratorType = MessageIteratorViewSynchronized<Chirality::right, decltype(msgs.begin()), decltype(lock_it)>;
      omega_iterator_with_lock<decltype(omegaBegin), decltype(lock_rec.cbegin())> omega_it(omegaBegin, lock_rec.cbegin()) ;
      MessageType::SendMessagesToLeft(rightFactor, MessageIteratorType(msgs.begin(), lock_rec.begin()), MessageIteratorType(msgs.end(), lock_rec.end()-1), omega_it);

      // unlock those factors which were locked above
      lock_it = lock_rec.begin();
      for(auto it=msgs.begin(); it!=msgs.end(); ++it, ++lock_it) {
        if(*lock_it) {
          (*it)->GetLeftFactor()->mutex_.unlock();
        }
      }
      assert(lock_it+1 == lock_rec.end());
   }
#endif
   template<typename RIGHT_FACTOR, typename MSG_ARRAY, typename ITERATOR>
   static void SendMessagesToLeftContainerSynchronized(const RIGHT_FACTOR& rightFactor, const MSG_ARRAY& msgs, ITERATOR omegaBegin) 
   {
     // record which factors were locked here
     if(msgs.size() <= 63) {
       std::array<bool,64> lock_rec;
       lock_rec[msgs.size()] = true;
       SendMessagesToLeftContainerSynchronized_impl(rightFactor, msgs, omegaBegin, lock_rec);
     } else {
       std::vector<bool> lock_rec(msgs.size()+1); // replace with own vector
       lock_rec[msgs.size()] = true;
       SendMessagesToLeftContainerSynchronized_impl(rightFactor, msgs, omegaBegin, lock_rec);
     }
   }

   constexpr static bool CanCallSendMessagesToRightContainer()
   {
      // possibly the below is to complicated. meta::find will be easier
      constexpr INDEX msg_array_number = LeftFactorContainer::template FindMessageDispatcherTypeIndex<MessageDispatcher<MessageContainerType,LeftMessageFuncGetter>>();
      using msg_container_type = meta::at_c<typename LeftFactorContainer::msg_container_type_list, msg_array_number>;
      using MSG_ARRAY_ITERATOR = decltype(std::declval<msg_container_type>().begin());
      return FunctionExistence::HasSendMessagesToRight<MessageType, void, LeftFactorType, MSG_ARRAY_ITERATOR, MSG_ARRAY_ITERATOR, REAL>();
   }

   // rename send_messages_to_right_container
   template<typename LEFT_FACTOR, typename MSG_ITERATOR>
   static void SendMessagesToRightContainer(const LEFT_FACTOR& leftFactor, MSG_ITERATOR msgs_begin, MSG_ITERATOR msgs_end, const REAL omega) 
   {
      using MessageIteratorType = MessageIteratorView<Chirality::left, MSG_ITERATOR>;
      return MessageType::SendMessagesToRight(leftFactor, MessageIteratorType(msgs_begin), MessageIteratorType(msgs_end), omega);
   }

#ifdef LP_MP_PARALLEL
   template<typename LEFT_FACTOR, typename MSG_ARRAY, typename ITERATOR, typename LOCK_VECTOR>
   static void SendMessagesToRightContainerSynchronized_impl(const LEFT_FACTOR& leftFactor, const MSG_ARRAY& msgs, ITERATOR omegaBegin, LOCK_VECTOR& lock_rec) 
   {
      // first lock as many adjacent factors as possible.
      auto lock_it = lock_rec.begin();
      for(auto it=msgs.begin(); it!=msgs.end(); ++it, ++lock_it) {
        if((*it)->GetRightFactor()->mutex_.try_lock()) { // mark that factor was locked by this process
          *lock_it = true;
        } else {
          *lock_it = false; 
#ifndef NDEBUG
          if(debug()) {
            std::cout << "locking failed for " << (*it)->GetRightFactor() << "\n";
          }
#endif
        }
      }
      assert(lock_it+1 == lock_rec.end());
      //std::fill(lock_rec.begin(), lock_rec.end(), true);

      using MessageIteratorType = MessageIteratorViewSynchronized<Chirality::left, decltype(msgs.begin()), decltype(lock_it)>;
      omega_iterator_with_lock<decltype(omegaBegin), decltype(lock_rec.cbegin())> omega_it(omegaBegin, lock_rec.cbegin()) ;
      MessageType::SendMessagesToRight(leftFactor, MessageIteratorType(msgs.begin(), lock_rec.begin()), MessageIteratorType(msgs.end(), lock_rec.end()-1), omega_it);

      // unlock those factors which were locked above
      lock_it = lock_rec.begin();
      for(auto it=msgs.begin(); it!=msgs.end(); ++it, ++lock_it) {
        if(*lock_it) {
          (*it)->GetRightFactor()->mutex_.unlock();
        }
      }
      assert(lock_it+1 == lock_rec.end());
   }
   template<typename LEFT_FACTOR, typename MSG_ARRAY, typename ITERATOR>
   static void SendMessagesToRightContainerSynchronized(const LEFT_FACTOR& leftFactor, const MSG_ARRAY& msgs, ITERATOR omegaBegin) 
   {
     // record which factors were locked here
     if(msgs.size() <= 63) {
       std::array<bool,64> lock_rec;
       lock_rec[msgs.size()] = true;
       SendMessagesToRightContainerSynchronized_impl(leftFactor, msgs, omegaBegin, lock_rec);
     } else {
       std::vector<bool> lock_rec(msgs.size()+1); // replace with own vector
       lock_rec[msgs.size()] = true;
       SendMessagesToRightContainerSynchronized_impl(leftFactor, msgs, omegaBegin, lock_rec);
     }
   }
#endif

   constexpr static bool
   CanComputeRightFromLeftPrimal()
   {
      return CanComputeRightFromLeftPrimalWithoutReturn() || CanComputeRightFromLeftPrimalWithReturn();
   }
   constexpr static bool
   CanComputeLeftFromRightPrimal()
   {
      return CanComputeLeftFromRightPrimalWithoutReturn() || CanComputeLeftFromRightPrimalWithReturn();
   } 

   constexpr static bool
   CanComputeRightFromLeftPrimalWithoutReturn()
   {
      return FunctionExistence::HasComputeRightFromLeftPrimal<MessageType, void, LeftFactorType, RightFactorType>();
   }
   constexpr static bool
   CanComputeLeftFromRightPrimalWithoutReturn()
   {
      return FunctionExistence::HasComputeLeftFromRightPrimal<MessageType, void, LeftFactorType, RightFactorType>();
   }

   constexpr static bool
   CanComputeRightFromLeftPrimalWithReturn()
   {
      return FunctionExistence::HasComputeRightFromLeftPrimal<MessageType, bool, LeftFactorType, RightFactorType>();
   }
   constexpr static bool
   CanComputeLeftFromRightPrimalWithReturn()
   {
      return FunctionExistence::HasComputeLeftFromRightPrimal<MessageType, bool, LeftFactorType, RightFactorType>();
   }

   void ComputeRightFromLeftPrimal() 
   {
      rightFactor_->conditionally_init_primal(leftFactor_->primal_access_);

      if constexpr (CanComputeRightFromLeftPrimalWithoutReturn()) {
        msg_op_.ComputeRightFromLeftPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
        rightFactor_->PropagatePrimal();
        rightFactor_->propagate_primal_through_messages(); 
      } else if constexpr (MessageContainerType::CanComputeRightFromLeftPrimalWithReturn()) {
        const bool changed = msg_op_.ComputeRightFromLeftPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
        if(changed) {
          rightFactor_->PropagatePrimal();
          rightFactor_->propagate_primal_through_messages();
        }
      }

      /*
      static_if<CanComputeRightFromLeftPrimalWithoutReturn()>([&](auto f) {
        f(msg_op_).ComputeRightFromLeftPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
        rightFactor_->PropagatePrimal();
        rightFactor_->propagate_primal_through_messages();
      }).else_([&](auto) {
         static_if<MessageContainerType::CanComputeRightFromLeftPrimalWithReturn()>([&](auto f) {
               const bool changed = f(msg_op_).ComputeRightFromLeftPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
               if(changed) {
                  rightFactor_->PropagatePrimal();
                  rightFactor_->propagate_primal_through_messages();
               }
         });
      });
      */
   }

   void ComputeLeftFromRightPrimal()
   {
      leftFactor_->conditionally_init_primal(rightFactor_->primal_access_);
      static_if<CanComputeLeftFromRightPrimalWithoutReturn()>([&](auto f) {
        f(msg_op_).ComputeLeftFromRightPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
        leftFactor_->PropagatePrimal();
        leftFactor_->propagate_primal_through_messages();
      }).else_([&](auto ) {
         static_if<MessageContainerType::CanComputeLeftFromRightPrimalWithReturn()>([&](auto f) {
               const bool changed = f(msg_op_).ComputeLeftFromRightPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
               if(changed) {
                  leftFactor_->PropagatePrimal();
                  leftFactor_->propagate_primal_through_messages();
               }
         });
      });
   }

   constexpr static bool
   CanCheckPrimalConsistency()
   {
      return FunctionExistence::HasCheckPrimalConsistency<MessageType,bool,
          typename LeftFactorContainer::FactorType,
          typename RightFactorContainer::FactorType>();
   }

   bool CheckPrimalConsistency() const final
   { 
      bool ret;
      static_if<CanCheckPrimalConsistency()>([&](auto f) {
            ret = f(msg_op_).CheckPrimalConsistency(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
      }).else_([&](auto) {
            ret = true;
      });
      return ret;
   }

   // do zrobienia: not needed anymore
   // do zrobienia: this does not capture write back functions not returning REAL&
   constexpr static bool IsAssignableLeft() {
      return FunctionExistence::IsAssignable<LeftFactorType, REAL, INDEX>();
   }
   constexpr static bool IsAssignableRight() {
      return FunctionExistence::IsAssignable<RightFactorType, REAL, INDEX>();
   }

   template<typename ARRAY, bool IsAssignable = IsAssignableLeft()>
   constexpr static bool CanBatchRepamLeft()
   {
      return FunctionExistence::HasRepamLeft<MessageType,void,LeftFactorType,ARRAY>();
   }
   template<typename ARRAY, bool IsAssignable = IsAssignableLeft()>
   //typename std::enable_if<CanBatchRepamLeft<ARRAY>() == true && IsAssignable == true>::type
   void
   RepamLeft(const ARRAY& m)
   { 
      //assert(false); // no -+ distinguishing
      static_if<CanBatchRepamLeft<ARRAY>()>([&](auto f) {
            f(msg_op_).RepamLeft(*(leftFactor_->GetFactor()), m);
      }).else_([&](auto f) {
         for(INDEX i=0; i<f(m).size(); ++i) {
            f(msg_op_).RepamLeft(*(leftFactor_->GetFactor()), f(m)[i], i);
         }
      });
   }
   /*
   template<typename ARRAY, bool IsAssignable = IsAssignableLeft()>
   typename std::enable_if<CanBatchRepamLeft<ARRAY>() == false && IsAssignable == true>::type
   RepamLeft(const ARRAY& m)
   { 
      //assert(false); // no -+ distinguishing
      for(INDEX i=0; i<m.size(); ++i) {
         msg_op_.RepamLeft(*(leftFactor_->GetFactor()), m[i], i);
      }
   }
   template<typename ARRAY, bool IsAssignable = IsAssignableLeft()>
   typename std::enable_if<IsAssignable == false>::type
   RepamLeft(const ARRAY& m)
   {
   assert(false);
   }
   */

   template<bool IsAssignable = IsAssignableLeft()>
   //typename std::enable_if<IsAssignable == true>::type
   void
   RepamLeft(const REAL diff, const INDEX dim) {
      msg_op_.RepamLeft(*(leftFactor_->GetFactor()), diff, dim); // note: in right, we reparametrize by +diff, here by -diff
   }
   /*
   template<bool IsAssignable = IsAssignableLeft()>
   typename std::enable_if<IsAssignable == false>::type
   RepamLeft(const REAL diff, const INDEX dim)
   {
   assert(false);
   }
   */

   template<typename ARRAY>
   constexpr static bool CanBatchRepamRight()
   {
      // do zrobienia: replace Container by actual factor
      return FunctionExistence::HasRepamRight<MessageType,void,RightFactorType,ARRAY>();
   }
   template<typename ARRAY, bool IsAssignable = IsAssignableRight()>
   //typename std::enable_if<CanBatchRepamRight<ARRAY>() == true && IsAssignable == true>::type
   void
   RepamRight(const ARRAY& m)
   { 
      //assert(false); // no -+ distinguishing
      static_if<CanBatchRepamRight<ARRAY>()>([&](auto f) {
            f(msg_op_).RepamRight(*(rightFactor_->GetFactor()), m);
      }).else_([&](auto f) {
         for(INDEX i=0; i<f(m).size(); ++i) {
            f(msg_op_).RepamRight(*(rightFactor_->GetFactor()), f(m)[i], i);
         }
      });
   }
   /*
   template<typename ARRAY, bool IsAssignable = IsAssignableRight()>
   typename std::enable_if<CanBatchRepamRight<ARRAY>() == false && IsAssignable == true>::type
   RepamRight(const ARRAY& m)
   {
      //assert(false); // no -+ distinguishing
      for(INDEX i=0; i<m.size(); ++i) {
         msg_op_.RepamRight(*(rightFactor_->GetFactor()), m[i], i);
      }
   }
   template<typename ARRAY, bool IsAssignable = IsAssignableRight()>
   typename std::enable_if<IsAssignable == false>::type
   RepamRight(const ARRAY& m)
   {
   assert(false);
   }
   */

   template<bool IsAssignable = IsAssignableRight()>
   //typename std::enable_if<IsAssignable == true>::type
   void
   RepamRight(const REAL diff, const INDEX dim) {
      msg_op_.RepamRight(*(rightFactor_->GetFactor()), diff, dim);
   }
   /*
   template<bool IsAssignable = IsAssignableRight()>
   typename std::enable_if<IsAssignable == false>::type
   RepamRight(const REAL diff, const INDEX dim)
   {
   assert(false);
   }
   */

   // do zrobienia: better name?
   //REAL GetLeftMessage(const INDEX i) const { return msg_op_.GetLeftMessage(i,*this); }
   //REAL GetRightMessage(const INDEX i) const { return msg_op_.GetRightMessage(i,*this);  }

   FactorTypeAdapter* GetLeftFactorTypeAdapter() const { return leftFactor_; }
   FactorTypeAdapter* GetRightFactorTypeAdapter() const { return rightFactor_; }
   // do zrobienia: Rename Get{Left|Right}FactorContainer
   LeftFactorContainer* GetLeftFactor() const final { return leftFactor_; }
   RightFactorContainer* GetRightFactor() const final { return rightFactor_; }

   void SetLeftFactor(FactorTypeAdapter* l) final 
   {
      assert(dynamic_cast<LeftFactorContainer*>(l));
      leftFactor_ = static_cast<LeftFactorContainer*>(l); 
   }
   void SetRightFactor(FactorTypeAdapter* r) final 
   {
      assert(dynamic_cast<RightFactorContainer*>(r));
      rightFactor_ = static_cast<RightFactorContainer*>(r); 
   }

   //INDEX GetMessageNumber() const final { return MESSAGE_NO; } 
   //REAL GetMessageWeightToRight() const final { return SEND_MESSAGE_TO_RIGHT_WEIGHT::value; }
   //REAL GetMessageWeightToLeft() const final { return SEND_MESSAGE_TO_LEFT_WEIGHT::value;  }
   
   // class for storing a callback upon new assignment of message: update left and right factors
   // convention is as follows: original message is for right factor. Inverted message is for left one
   template<Chirality CHIRALITY>
   class MsgVal {
   public:
      MsgVal(MessageContainerType* msg, const INDEX dim) : 
         msg_(msg), 
         dim_(dim)
      {}
      // do zrobienia: do not support this operation! Goal is to not hold messages anymore, except for implicitly held reparametrizations.
      /*
      MsgVal& operator=(const REAL x) __attribute__ ((always_inline))
      {
         assert(false);
         const REAL diff = x - msg_->operator[](dim_);
         // set new message
         static_cast<typename MessageContainerType::MessageStorageType*>(msg_)->operator[](dim_) = x;
         // propagate difference to left and right factor
         msg_->RepamLeft( diff, dim_);
         msg_->RepamRight( diff, dim_);
         return *this;
      }
      */
      MsgVal& operator-=(const REAL x) __attribute__ ((always_inline))
      {
         if(CHIRALITY == Chirality::right) { // message is computed by right factor
            msg_->RepamLeft( +x, dim_);
            msg_->RepamRight(-x, dim_);
         } else if (CHIRALITY == Chirality::left) { // message is computed by left factor
            msg_->RepamLeft(  -x, dim_);
            msg_->RepamRight( +x, dim_);
            //msg_->RepamLeft( +x, dim_);
            //msg_->RepamRight( +x, dim_);
         } else {
            assert(false);
         }
         return *this;
      }
      MsgVal& operator+=(const REAL x) __attribute__ ((always_inline))
      {
         assert(false);
         if(CHIRALITY == Chirality::right) { // message is computed by right factor
            msg_->RepamLeft( x, dim_);
            msg_->RepamRight( x, dim_);
         } else if(CHIRALITY == Chirality::left) { // message is computed by left factor
            msg_->RepamLeft( x, dim_);
            msg_->RepamRight( x, dim_);
            //msg_->RepamLeft( -x, dim_);
            //msg_->RepamRight( -x, dim_);
         } else {
            assert(false);
         }
         return *this;
      }
      // do zrobienia: this value should never be used. Remove function
      //operator REAL() const __attribute__ ((always_inline)) { return static_cast<typename MessageContainerType::MessageStorageType*>(msg_)->operator[](dim_); }
   private:
      MessageContainerType* const msg_;
      const INDEX dim_;
   };

   // this view of the message container is given to left and right factor respectively when receiving or sending messages
   template<Chirality CHIRALITY>
   class MessageContainerView : public MessageContainerType {
   public:
      //using MessageContainerType;
      MsgVal<CHIRALITY> operator[](const INDEX i) 
      {
         return MsgVal<CHIRALITY>(this,i);
      }

      template<typename ARRAY>
      MessageContainerType& operator-=(const ARRAY& diff) {
        // note: order of below operations is important: When the message is e.g. just the potential, we must reparametrize the other side first!
        if(CHIRALITY == Chirality::right) {
          RepamLeft(diff);
          RepamRight(-diff);
        } else if(CHIRALITY == Chirality::left) {
          RepamRight(diff);
          RepamLeft(-diff);
        } else {
          assert(false);
        }
        return *this;
      }

   };

   // for primal computation: record message change only in one side and into a special array
   template<Chirality CHIRALITY>
   class OneSideMsgVal
   {
   public:
      OneSideMsgVal(MessageContainerType* msg, const INDEX dim) : 
         msg_(msg), 
         dim_(dim)
      {}

      OneSideMsgVal& operator-=(const REAL x) __attribute__ ((always_inline))
      {
         if(CHIRALITY == Chirality::right) { // message is received by right factor
            msg_->RepamRight(+x, dim_);
         } else if (CHIRALITY == Chirality::left) { // message is received by left factor
            msg_->RepamLeft(+x, dim_);
         } else {
            assert(false);
         }
         return *this;
      }

      OneSideMsgVal& operator+=(const REAL x) __attribute__ ((always_inline))
      {
         assert(false);
         if(CHIRALITY == Chirality::right) {
            msg_->RepamRight(-x, dim_);
         } else if(CHIRALITY == Chirality::left) {
            msg_->RepamLeft(-x, dim_);
         } else {
            assert(false);
         }
         return *this;
      }

   private:
      MessageContainerType* const msg_;
      const INDEX dim_;
   };

   // this view is given to receive restricted message operations. 
   // Reparametrization is recorded only on one side
   template<Chirality CHIRALITY>
   class OneSideMessageContainerView : public MessageContainerType{
   public:
      //using MessageContainerType;
      OneSideMsgVal<CHIRALITY> operator[](const INDEX i) 
      {
         return OneSideMsgVal<CHIRALITY>(this,i);
      }

      template<typename ARRAY>
      MessageContainerType& operator-=(const ARRAY& diff) {
        if(CHIRALITY == Chirality::right) {
          RepamRight(diff);
        } else if(CHIRALITY == Chirality::left) {
          RepamLeft(diff);
        } else {
          assert(false);
        }
        return *this;
      }

   };


   // there must be four different implementations of msg updating with SIMD: 
   // (i) If parallel reparametrization is not supported by either left and right factor
   // If (ii) left or (iii) right factor supports reparametrization but not both
   // If (iv) both left and right factor support reparametrization


   template<typename ARRAY>
   MessageContainerType& operator-=(const ARRAY& diff) {
      assert(false); // update to left right -+
      RepamLeft(-diff);
      RepamRight(-diff);
      return *this;
   }

   template<typename ARRAY>
   MessageContainerType& operator+=(const ARRAY& diff) {
      assert(false); // update to left right -+
      RepamLeft(diff);
      RepamRight(diff);
      return *this;
   }

   // possibly not the best choice: Sometimes msg_op_ needs access to this class
   const MessageType& GetMessageOp() const
   {
      return msg_op_;
   }

   // for weight computations these functions are necessary
   static constexpr bool sends_message_to_left_constexpr()
   {
      return MPS == message_passing_schedule::right || MPS == message_passing_schedule::full || MPS == message_passing_schedule::only_send;
   }
   static constexpr bool sends_message_to_right_constexpr() 
   {
      return MPS == message_passing_schedule::left || MPS == message_passing_schedule::full || MPS == message_passing_schedule::only_send;
   }
   static constexpr bool receives_message_from_left_constexpr() 
   {
     return MPS == message_passing_schedule::right || MPS == message_passing_schedule::full;
   }
   static constexpr bool receives_message_from_right_constexpr() 
   {
     return MPS == message_passing_schedule::left || MPS == message_passing_schedule::full;
   }

   virtual bool SendsMessageToLeft() const final { return sends_message_to_left_constexpr(); }
   virtual bool SendsMessageToRight() const final { return sends_message_to_right_constexpr(); }
   virtual bool ReceivesMessageFromLeft() const final { return receives_message_from_left_constexpr(); }
   virtual bool ReceivesMessageFromRight() const final { return receives_message_from_right_constexpr(); }

   /*
   constexpr static bool
   can_construct_constraints()
   {
      return has_construct_sat_clauses<MessageType,void,sat_solver&, LeftFactorType, RightFactorType, sat_var, sat_var>();
   }

   void construct_sat_clauses(sat_solver& s, sat_var left_var, sat_var right_var) 
   {
      static_if<can_construct_sat_clauses()>([&](auto f) {
            f(msg_op_).construct_sat_clauses(s, *leftFactor_->GetFactor(), *rightFactor_->GetFactor(), left_var, right_var);
            });
      if(!can_construct_sat_clauses()) {
         assert(false);
      }
   }
   */ 
   
   // for traversing a tree
   virtual void send_message_up(Chirality c) final
   {
      if(c == Chirality::right) { // right factor is top one
         leftFactor_->GetFactor()->init_primal();
         this->send_message_to_right();
         //static_if<CanCallReceiveMessageFromLeftContainer()>([&](auto f) {
         //      f(this)->ReceiveMessageFromLeftContainer();
         //}).else_([&](auto) {
         //      static_if<MessageContainerType::CanCallSendMessageToRightContainer()>([&](auto f) {
         //               f(this)->SendMessageToRightContainer(leftFactor_->GetFactor(),1.0);
         //      }).else_([](auto) {
         //         assert(false); // possibly try to call SendMessagesToRightContainer with exactly one message
         //      });
         //});
      } else {
         rightFactor_->GetFactor()->init_primal();
         this->send_message_to_left();
         //static_if<CanCallReceiveMessageFromRightContainer()>([&](auto f) {
         //      f(this)->ReceiveMessageFromRightContainer();
         //}).else_([&](auto) {
         //      static_if<MessageContainerType::CanCallSendMessageToLeftContainer()>([&](auto f) {
         //               f(this)->SendMessageToLeftContainer(rightFactor_->GetFactor(),1.0);
         //      }).else_([](auto) {
         //         assert(false); // possibly try to call SendMessagesToRightContainer with exactly one message
         //      });
         //});
      }
   }

   
   virtual void track_solution_down(Chirality c) final
   {
      // we can assume that upper factor has already (partially) computed primal.
      // we check whether we can receive restricted messages from upper and compute primal in lower. If yes, we receive restricted message, compute primal in lower factor and propagate it back to upper factor.
      if(c == Chirality::right) {
         //leftFactor_->init_primal(); // initialization is already done in upward pass
         static_if<MessageContainerType::CanComputeLeftFromRightPrimal()>([&](auto f) {
               f(msg_op_).ComputeLeftFromRightPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
         });
         static_if<LeftFactorContainer::CanMaximizePotentialAndComputePrimal()>([&](auto f2) {
               f2(leftFactor_)->MaximizePotentialAndComputePrimal(); 
         });
      } else {
         assert(c == Chirality::left);
         //rightFactor_->init_primal();
         static_if<MessageContainerType::CanComputeRightFromLeftPrimal()>([&](auto f) {
               f(msg_op_).ComputeRightFromLeftPrimal(*leftFactor_->GetFactor(), *rightFactor_->GetFactor());
         });
         static_if<RightFactorContainer::CanMaximizePotentialAndComputePrimal()>([&](auto f2) {
               f2(rightFactor_)->MaximizePotentialAndComputePrimal(); 
         });
      }
      return;


      // if this is not possible, we propagate primal labeling of upper to lower
      if(c == Chirality::right) { // right factor is upper
         static_if<LeftFactorContainer::CanMaximizePotentialAndComputePrimal() && CanCallReceiveRestrictedMessageFromRightContainer()>([&](auto f) {
                  // receive restricted messages 
                  serialization_archive ar(leftFactor_->GetFactor(), leftFactor_->GetFactor()+1, [](auto& f, auto& ar) { f.serialize_dual(ar); });
                  save_archive s_ar(ar);
                  leftFactor_->GetFactor()->serialize_dual( s_ar );

                  f(this)->ReceiveRestrictedMessageFromRightContainer();

                  // compute primal in lower
                  leftFactor_->MaximizePotentialAndComputePrimal();

                  // restore dual reparametrization to before restricted messages were sent.
                  load_archive l_ar(ar);
                  leftFactor_->GetFactor()->serialize_dual( l_ar );

                  // propagate back to upper
                  f(this)->ComputeRightFromLeftPrimal(); 

         }).else_([&](auto) {
            static_if<MessageContainerType::CanComputeLeftFromRightPrimal()>([&](auto f) {
                  f(this)->ComputeLeftFromRightPrimal();
                  static_if<LeftFactorContainer::CanMaximizePotentialAndComputePrimal()>([&](auto f2) {
                        f2(leftFactor_)->MaximizePotentialAndComputePrimal(); 
                  });
            }).else_([&](auto) {
               assert(false);
            });
         });

      } else if(c == Chirality::left) { // left factor is upper
         static_if<RightFactorContainer::CanMaximizePotentialAndComputePrimal() && CanCallReceiveRestrictedMessageFromLeftContainer()>([&](auto f) {
                  std::stringstream ss;
                  // receive restricted messages 
                  serialization_archive ar(rightFactor_->GetFactor(), rightFactor_->GetFactor()+1, [](auto& f, auto& ar) { f.serialize_dual(ar); });
                  save_archive s_ar(ar);
                  rightFactor_->GetFactor()->serialize_dual( s_ar );

                  f(this)->ReceiveRestrictedMessageFromLeftContainer();

                  // compute primal in lower
                  rightFactor_->MaximizePotentialAndComputePrimal();

                  // restore dual reparametrization to before restricted messages were sent.
                  load_archive l_ar(ar);
                  rightFactor_->GetFactor()->serialize_dual( l_ar );

                  // propagate back to upper
                  f(this)->ComputeLeftFromRightPrimal(); 

         }).else_([&](auto) {
            static_if<MessageContainerType::CanComputeRightFromLeftPrimal()>([&](auto f) {
                  f(this)->ComputeRightFromLeftPrimal();
                  static_if<RightFactorContainer::CanMaximizePotentialAndComputePrimal()>([&](auto f2) {
                        f2(rightFactor_)->MaximizePotentialAndComputePrimal(); 
                  });
            }).else_([&](auto) {
               assert(false);
            });
         });

      } else {
         assert(false);
      }
   }

   // construct constraints
   template<typename SOLVER>
   void construct_constraints_impl(SOLVER& s, const typename DD_ILP::variable_counters& left_variable_counters, const typename DD_ILP::variable_counters& right_variable_counters)
   {
      auto current_variable_counters = s.get_variable_counters();

      auto left_vars = leftFactor_->GetFactor()->export_variables();
      s.set_variable_counters(left_variable_counters);
      auto left_external_vars = std::apply([this,&s](auto... x){ return std::make_tuple(this->leftFactor_->load_external_variables(s, x)...); }, left_vars);

      auto right_vars = rightFactor_->GetFactor()->export_variables();
      s.set_variable_counters(right_variable_counters);
      auto right_external_vars = std::apply([this,&s](auto... x){ return std::make_tuple(this->rightFactor_->load_external_variables(s, x)...); }, right_vars);

      auto t = std::tuple_cat(std::tie(*leftFactor_->GetFactor()), left_external_vars, std::tie(*rightFactor_->GetFactor()), right_external_vars);
      auto construct_constraints_fun = [this,&s](auto... x) { this->msg_op_.construct_constraints(s, x...); };
      std::apply(construct_constraints_fun, t);

      s.set_variable_counters(current_variable_counters);
   }

   virtual void construct_constraints(
       DD_ILP::external_solver_interface<DD_ILP::sat_solver>& s, 
       const typename DD_ILP::variable_counters& left_variable_counters,
       const typename DD_ILP::variable_counters& right_variable_counters 
       ) final
   { construct_constraints_impl(s, left_variable_counters, right_variable_counters); }
   virtual void construct_constraints(
       DD_ILP::external_solver_interface<DD_ILP::problem_export>& s, 
       const typename DD_ILP::variable_counters& left_variable_counters,
       const typename DD_ILP::variable_counters& right_variable_counters 
       ) final
   { construct_constraints_impl(s, left_variable_counters, right_variable_counters); }
#ifdef DD_ILP_WITH_GUROBI
   virtual void construct_constraints(
       DD_ILP::external_solver_interface<DD_ILP::gurobi_interface>& s, 
       const typename DD_ILP::variable_counters& left_variable_counters,
       const typename DD_ILP::variable_counters& right_variable_counters 
       ) final
   { construct_constraints_impl(s, left_variable_counters, right_variable_counters); }
#endif


protected:
   MessageType msg_op_; // possibly inherit privately from MessageType to apply empty base optimization when applicable
   LeftFactorContainer* leftFactor_;
   RightFactorContainer* rightFactor_;

   // see notes on allocator in FactorContainer
   struct Allocator {
      using type = MemoryPool<MessageContainerType,4096*(sizeof(MessageContainerType)+sizeof(void*))>; 
      static type& get() {
         static type allocator;
         return allocator;
      }
   };
};


// container class for factors. Here we hold the factor, all connected messages, reparametrization storage and perform reparametrization and coordination for sending and receiving messages.
// derives from REPAM_STORAGE_TYPE to mixin a class for storing the reparametrized potential
// implements the interface from FactorTypeAdapter for access from LP_MP
// if COMPUTE_PRIMAL_SOLUTION is true, MaximizePotential is expected to return either an integer of type INDEX or a std::vector<INDEX>
// if WRITE_PRIMAL_SOLUTION is false, WritePrimal will not output anything
// do zrobienia: introduce enum classes for COMPUTE_PRIMAL_SOLUTION and WRITE_PRIMAL_SOLUTION
template<typename FACTOR_TYPE, 
         class FACTOR_MESSAGE_TRAIT,
         INDEX FACTOR_NO,
         bool COMPUTE_PRIMAL_SOLUTION = false> 
class FactorContainer : public FactorTypeAdapter
{
public:
   using FactorContainerType = FactorContainer<FACTOR_TYPE, FACTOR_MESSAGE_TRAIT, FACTOR_NO, COMPUTE_PRIMAL_SOLUTION>;
   using FactorType = FACTOR_TYPE;

   // do zrobienia: templatize cosntructor to allow for more general initialization of reparametrization storage and factor
   template<typename ...ARGS>
   FactorContainer(ARGS... args) : factor_(args...) {}

   FactorContainer(const FactorType&& factor) : factor_(std::move(factor)) {
      //INDEX status;
      //std::cout << "msg_ type= "  << abi::__cxa_demangle(typeid(msg_).name(),0,0,&status) << "\n";
      //std::cout << "dispatcher list = "  << abi::__cxa_demangle(typeid(MESSAGE_DISPATCHER_TYPELIST).name(),0,0,&status) << "\n";
      //std::cout << "msg_ type= "  << abi::__cxa_demangle(typeid(msg_).name(),0,0,&status) << "\n";
      //std::cout << "left message list = " << abi::__cxa_demangle(typeid(left_message_list_).name(),0,0,&status) << "\n";
      //std::cout << "left message list = " << abi::__cxa_demangle(typeid(left_message_list_1).name(),0,0,&status) << "\n";
   
   }
   FactorContainer(const FactorType& factor) : factor_(factor) 
   {}
   ~FactorContainer() { 
      static_assert(meta::unique<MESSAGE_DISPATCHER_TYPELIST>::size() == MESSAGE_DISPATCHER_TYPELIST::size(), 
            "Message dispatcher typelist must have unique elements");
      static_assert(FACTOR_NO >= 0 && FACTOR_NO < FACTOR_MESSAGE_TRAIT::FactorList::size(), "factor number must be smaller than length of factor list");
   }

   // overloaded new so that factor containers are allocated by global block allocator consecutively
   void* operator new(std::size_t size)
   {
      assert(size == sizeof(FactorContainerType));
      //INDEX s = size/sizeof(REAL);
      //if(size % sizeof(REAL) != 0) { s++; }
      //return (void*) global_real_block_allocator.allocate(s,1);
      return Allocator::get().allocate(1);
   }
   void operator delete(void* mem)
   {
      Allocator::get().deallocate((FactorContainerType*) mem);
      //assert(false);
      //global_real_block_allocator.deallocate((double*)mem,sizeof(FactorContainerType)/sizeof(REAL)+1);
   }

   virtual FactorTypeAdapter* clone() const final
   {
      auto* c = new FactorContainer(factor_);
      return c;
   }

   template<typename MESSAGE_DISPATCHER_TYPE, typename MESSAGE_TYPE> 
   void AddMessage(MESSAGE_TYPE* m) { 
      constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<MESSAGE_DISPATCHER_TYPE>();
      static_assert( n < meta::size<MESSAGE_DISPATCHER_TYPELIST>(), "message dispatcher not supported");
      static_assert( n < std::tuple_size<decltype(msg_)>(), "message dispatcher not supported");
      //INDEX status;
      //std::cout << "msg dispatcher list =\n" << abi::__cxa_demangle(typeid(MESSAGE_DISPATCHER_TYPELIST).name(),0,0,&status) << "\n";
      //std::cout << "dispatcher  type =\n" << abi::__cxa_demangle(typeid(MESSAGE_DISPATCHER_TYPE).name(),0,0,&status) << "\n";
      //std::cout << " number = " << n << "\n" ;
      //std::cout << "message type = " << abi::__cxa_demangle(typeid(MESSAGE_TYPE).name(),0,0,&status) << "\n";

      std::get<n>(msg_).push_back(m);
   }

   void UpdateFactor(const weight_vector& omega) final
   {
      assert(*std::min_element(omega.begin(), omega.end()) >= 0.0);
      assert(std::accumulate(omega.begin(), omega.end(), 0.0) <= 1.0 + eps);
      assert(std::distance(omega.begin(), omega.end()) == no_send_messages());
      ReceiveMessages(omega);
      MaximizePotential();
      SendMessages(omega);
   }

   void update_factor_residual(const weight_vector& omega)
   {
      assert(*std::min_element(omega.begin(), omega.end()) >= 0.0);
      assert(*std::max_element(omega.begin(), omega.end()) <= 1.0+eps);
      assert(std::distance(omega.begin(), omega.end()) == no_send_messages());
      ReceiveMessages(omega);
      MaximizePotential();
      send_messages_residual(omega); // other message passing type shall be called "shared"
   }

#ifdef LP_MP_PARALLEL
   void UpdateFactorSynchronized(const weight_vector& omega) final
   {
      assert(*std::min_element(omega.begin(), omega.end()) >= 0.0);
      assert(std::accumulate(omega.begin(), omega.end(), 0.0) <= 1.0 + eps);
      assert(std::distance(omega.begin(), omega.end()) == no_send_messages());
      std::lock_guard<std::recursive_mutex> lock(mutex_); // only here do we wait for the mutex. In all other places try_lock is allowed only
      ReceiveMessagesSynchronized(omega);
      MaximizePotential();
      SendMessagesSynchronized(omega);
   }

   void UpdateFactorPrimalSynchronized(const weight_vector& omega, const INDEX iteration) final
   {
     //std::cout << "not implemented\n";
     //assert(false);
   }
#endif

   // do zrobienia: possibly also check if method present
   constexpr static bool
   CanComputePrimal()
   {
      return COMPUTE_PRIMAL_SOLUTION;
   }

   constexpr static bool
   CanMaximizePotentialAndComputePrimal()
   {
      return FunctionExistence::HasMaximizePotentialAndComputePrimal<FactorType,void>();
   }

   constexpr static bool
   CanPropagatePrimal()
   {
      return FunctionExistence::HasPropagatePrimal<FactorType,void>();
   }

   void PropagatePrimal() 
   {
      static_if<CanPropagatePrimal()>([&](auto f) {
            f(factor_).PropagatePrimal();
      });
   }

   constexpr static bool
   CanMaximizePotential()
   {
      return FunctionExistence::HasMaximizePotential<FactorType,void>();
   }


   void UpdateFactorPrimal(const weight_vector& omega, INDEX primal_access) final
   {
#ifdef LP_MP_PARALLEL
     std::lock_guard<std::recursive_mutex> lock(mutex_); // only here do we wait for the mutex. In all other places try_lock is allowed only
#endif
      assert(primal_access > 0); // otherwise primal is not initialized in first iteration
      conditionally_init_primal(primal_access);
      if(CanComputePrimal()) { // do zrobienia: for now
         primal_access_ = primal_access;
         if(CanReceiveRestrictedMessage() && ReceivesRestrictedMessage()) {

            serialization_archive ar(GetFactor(), GetFactor()+1, [](auto& f, auto& ar) { f.serialize_dual(ar); });
            save_archive s_ar(ar);
            factor_.serialize_dual( s_ar );

            // now we change the dual information
            // first we compute restricted incoming messages, on which to compute the primal
            ReceiveRestrictedMessages();

            // now we compute primal w.r.t. the changed dual information!
            MaximizePotentialAndComputePrimal();

            // restore dual reparametrization to before restricted messages were sent.
            load_archive l_ar(ar);
            factor_.serialize_dual( l_ar );

            ReceiveMessages(omega);
            MaximizePotential();
            SendMessages(omega);
         } else {
            ReceiveMessages(omega);
            MaximizePotentialAndComputePrimal();
            SendMessages(omega);
         }
         // now propagate primal to adjacent factors
         propagate_primal_through_messages();
      } else {
         ReceiveMessages(omega);
         MaximizePotential();
         SendMessages(omega);
      }  
   }

   void MaximizePotential()
   {
      static_if<CanMaximizePotential()>([&](auto f) {
            f(factor_).MaximizePotential();
      });
   }

   virtual void MaximizePotentialAndComputePrimal() final
   {
      static_if<CanMaximizePotentialAndComputePrimal()>([&](auto f) {
            f(factor_).MaximizePotentialAndComputePrimal();
      });
      if(!CanMaximizePotentialAndComputePrimal()) { assert(false); }
   }

   // do zrobienia: rename PropagatePrimalThroughMessages
   virtual void propagate_primal_through_messages() final
   {
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this](auto l) {
            static_if<l.CanComputePrimalThroughMessage()>([&](auto f) {
                  constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
                  for(auto it = std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it) {
                     f(l).ComputePrimalThroughMessage(*(*it));
                  }
            });
      });
   }

   template<typename WEIGHT_VEC>
   void ReceiveMessages(const WEIGHT_VEC& omega) 
   {
      // note: currently all messages are received, even if not needed. Change this again.
      auto omegaIt = omega.begin();
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&omegaIt](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            static_if<l.ReceivesMessage()>([&](auto f) {
                  
                  for(auto it = std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it, ++omegaIt) {
                     //if(*omegaIt == 0.0) { // makes large difference for cosegmentation_bins, why?
                     f(l).ReceiveMessage(*(*it));
                     //}
                  }

                  });
            
            //std::advance(omegaIt, std::get<n>(msg_).size());
      });
   }

#ifdef LP_MP_PARALLEL
   template<typename WEIGHT_VEC>
   void ReceiveMessagesSynchronized(const WEIGHT_VEC& omega) 
   {
      // note: currently all messages are received, even if not needed. Change this again.
      auto omegaIt = omega.begin();
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&omegaIt](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            static_if<l.ReceivesMessage()>([&](auto f) {
                  
                  for(auto it = std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it, ++omegaIt) {
                     //if(*omegaIt == 0.0) { // makes large difference for cosegmentation_bins, why?
                     f(l).ReceiveMessageSynchronized(*(*it));
                     //}
                  }

                  });
            
            //std::advance(omegaIt, std::get<n>(msg_).size());
      });
   }
#endif

   // we write message change not into original reparametrization, but into temporary one named pot
   void ReceiveRestrictedMessages() 
   {
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            static_if<l.CanCallReceiveRestrictedMessage()>([&](auto f) {
                  for(auto it=std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it) {
                     f(l).ReceiveRestrictedMessage(*(*it)); 
                  }
            });
      });
   }

   struct can_receive_restricted_message {
      template<typename MESSAGE_DISPATCHER_TYPE>
         using invoke = typename std::is_same<std::integral_constant<bool,MESSAGE_DISPATCHER_TYPE::CanCallReceiveRestrictedMessage()>, std::integral_constant<bool,true> >::type;
   };
   constexpr static bool CanReceiveRestrictedMessage() 
   {
      return meta::any_of<MESSAGE_DISPATCHER_TYPELIST, can_receive_restricted_message>{};
   }

   // methods used by MessageIterator
   INDEX no_messages() const final
   {
      INDEX no_msgs = 0;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&no_msgs](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            no_msgs += std::get<n>(msg_).size();
            } );
      return no_msgs;
   }

   // counts number of messages for which messages are sent
   INDEX no_send_messages() const final
   {
      INDEX no_calls = 0;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&no_calls](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            if(l.SendsMessage()) {
               no_calls += std::get<n>(msg_).size();
            }
            } );
      return no_calls;
   }

   // as above, but if batch messages sending is enabled, such messages are counted only once.
   INDEX no_send_messages_calls() const 
   {
      INDEX no_calls = 0;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&no_calls](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            if(l.CanCallSendMessages() && l.SendsMessage()) {
               if(std::get<n>(msg_).size() > 0) {
                  ++no_calls;
               }
            } else if(l.SendsMessage()) {
               no_calls += std::get<n>(msg_).size();
            }
            } );
      return no_calls;
   }

   template<typename ITERATOR>
   void CallSendMessages(FactorType& factor, ITERATOR omegaIt) 
   {
     auto omega_begin = omegaIt;
     meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [&](auto l) {
         // check whether the message supports batch updates. If so, call batch update, else call individual send message
         constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
         if constexpr(l.SendsMessage()) {
           auto msg_begin = std::get<n>(msg_).begin();
           auto msg_end = std::get<n>(msg_).end();
           const INDEX no_messages = std::get<n>(msg_).size();

           if constexpr(l.CanCallSendMessages()) {

             const INDEX no_active_messages = std::count_if(omegaIt, omegaIt+no_messages, [](const REAL x){ return x > 0.0; });

             if(no_active_messages > 0) { 
               const REAL omega_sum = std::accumulate(omegaIt, omegaIt+no_messages, 0.0);
               // build array of messages that are actually called
               if(no_active_messages <= active_messages_array_size) {
                 auto active_msgs = get_active_messages_array(msg_begin, msg_end, omegaIt);
                 l.SendMessages(factor, active_msgs.begin(), active_msgs.begin()+no_active_messages, omega_sum);
               } else {
                 auto active_msgs = get_active_messages_vector(msg_begin, msg_end, omegaIt, no_active_messages);
                 l.SendMessages(factor, active_msgs.begin(), active_msgs.begin()+no_active_messages, omega_sum);
               }
             } 
             omegaIt += no_messages;

           } else {
             for(auto msg_it = msg_begin; msg_it != msg_end; ++msg_it, ++omegaIt) {
               if(*omegaIt != 0.0) {
                 l.SendMessage(&factor, *(*msg_it), *omegaIt); 
               }
             }
           }
         }
     });
     assert(omegaIt - omega_begin == no_send_messages());
   }

#ifdef LP_MP_PARALLEL
   template<typename ITERATOR>
   void CallSendMessagesSynchronized(FactorType& factor, ITERATOR omegaIt) 
   {
     meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [&](auto l) {
         // check whether the message supports batch updates. If so, call batch update.
         // If not, check whether individual updates are supported. If yes, call individual updates. If no, do nothing
         if(l.SendsMessage()) {
         static_if<l.CanCallSendMessages()>([&](auto f) {
             constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
             const REAL omega_sum = std::accumulate(omegaIt, omegaIt + std::get<n>(msg_).size(), 0.0);
             if(omega_sum > 0.0) { 
               f(l).SendMessagesSynchronized(factor, std::get<n>(msg_), omegaIt);
             }
             omegaIt += std::get<n>(msg_).size();
          }).else_([&](auto) {
                constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
                for(auto it = std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it, ++omegaIt) {
                  if(*omegaIt != 0.0) {
                    l.SendMessageSynchronized(&factor, *(*it), *omegaIt); 
                  }
                }
          });
         }
     });
   }
#endif

   template<typename WEIGHT_VEC>
   void SendMessages(const WEIGHT_VEC& omega) 
   {
      // do zrobienia: condition no_send_messages_calls also on omega. whenever omega is zero, we will not send messages
      const INDEX no_calls = no_send_messages_calls();

      if(no_calls == 1) {
        CallSendMessages(factor_, omega.begin());
      } else if( no_calls > 1 ) {
         // make a copy of the current reparametrization. The new messages are computed on it. Messages are updated implicitly and hence possibly the new reparametrization is automatically adjusted, which would interfere with message updates
         FactorType tmp_factor(factor_);

         CallSendMessages(tmp_factor, omega.begin());
      } else {
        assert(omega.size() == 0.0);
      }
   } 

   static constexpr INDEX active_messages_array_size = 16;

   template<typename MSG_ITERATOR, typename ACTIVE_ITERATOR>
   auto get_active_messages_array(MSG_ITERATOR msgs_begin, MSG_ITERATOR msgs_end, ACTIVE_ITERATOR active_begin)
   {
     using message_ptr_type = decltype( *msgs_begin );
     std::array<message_ptr_type,active_messages_array_size> msgs;
     for(auto [no_msgs,msg_it]=std::pair(0,msgs_begin); msg_it!=msgs_end; ++msg_it, ++active_begin) {
       if(*active_begin > 0.0) {
         msgs[no_msgs++] = *msg_it;
       }
     }
     return msgs; 
   }

   template<typename MSG_ITERATOR, typename ACTIVE_ITERATOR>
   auto get_active_messages_vector(MSG_ITERATOR msgs_begin, MSG_ITERATOR msgs_end, ACTIVE_ITERATOR active_begin, const INDEX no_active_messages)
   {
     using message_ptr_type = decltype( *msgs_begin );
     std::vector<message_ptr_type> msgs(no_active_messages);
     INDEX no_msgs = 0;
     for(auto msg_it=msgs_begin; msg_it!=msgs_end; ++msg_it, ++active_begin) {
       if(*active_begin > 0.0) {
         msgs[no_msgs++] = *msg_it;
       }
     }
     assert(no_active_messages == no_msgs); 
     return msgs; 
   }

   // choose order of messages to be sent and immediately reparametrize after each send message call and increase the remaining weights
   template<typename WEIGHT_VEC>
   void send_messages_residual(const WEIGHT_VEC& omega)
   {
     auto omegaIt = omega.begin();
     REAL residual_omega = 0.0;
     meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [&](auto l) {
         // check whether the message supports batch updates. If so, call batch update.
         // If not, check whether individual updates are supported. If yes, call individual updates. If no, do nothing
         constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
         using message_ptr_type = decltype( *(std::get<n>(msg_).begin()) );
         if constexpr(l.SendsMessage()) {
           const INDEX no_messages = std::get<n>(msg_).size();
           if(no_messages > 0) {

             if constexpr(l.CanCallSendMessages()) {
               const INDEX no_active_messages = std::count_if(omegaIt, omegaIt+no_messages, [](REAL x){ return x > 0.0; });
               const REAL omega_sum = std::accumulate(omegaIt, omegaIt+no_messages, 0.0);
               residual_omega += omega_sum;
               if(no_active_messages > 0) { 
                 // build array of messages that are actually called
                 if(no_active_messages <= active_messages_array_size) {
                   auto msgs = get_active_messages_array(std::get<n>(msg_).begin(), std::get<n>(msg_).end(), omegaIt);
                   l.SendMessages(factor_, msgs.begin(), msgs.begin()+no_active_messages, residual_omega);
                 } else {
                   auto msgs = get_active_messages_vector(std::get<n>(msg_).begin(), std::get<n>(msg_).end(), omegaIt, no_active_messages);
                   l.SendMessages(factor_, msgs.begin(), msgs.begin()+no_active_messages, residual_omega);
                 }
               } 
               omegaIt += no_messages;
  
             } else {
  
               for(auto it = std::get<n>(msg_).begin(); it != std::get<n>(msg_).end(); ++it, ++omegaIt) {
                 if(*omegaIt != 0.0) {
                   residual_omega += *omegaIt;
                   l.SendMessage(&factor_, *(*it), residual_omega); 
                 }
               }
             }
           }

         }
         assert(0.0 <= residual_omega && residual_omega <= 1.0 + eps);

       });
   }

#ifdef LP_MP_PARALLEL
   template<typename WEIGHT_VEC>
   void SendMessagesSynchronized(const WEIGHT_VEC& omega) 
   {
      // do zrobienia: condition no_send_messages_calls also on omega. whenever omega is zero, we will not send messages
      const INDEX no_calls = no_send_messages_calls();

      if(no_calls == 1) {
        CallSendMessagesSynchronized(factor_, omega.begin());
      } else if( no_calls > 1 ) {
         // make a copy of the current reparametrization. The new messages are computed on it. Messages are updated implicitly and hence possibly the new reparametrization is automatically adjusted, which would interfere with message updates
         FactorType tmp_factor(factor_);

         CallSendMessagesSynchronized(tmp_factor, omega.begin());
      } else {
        assert(omega.size() == 0.0);
      }
   } 
#endif

   template<typename ...MESSAGE_DISPATCHER_TYPES_REST>
   MessageTypeAdapter* GetMessage(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX) const 
   {
      assert(false);
      throw std::runtime_error("index out of bound");
      return nullptr;
   }

   template<typename MESSAGE_DISPATCHER_TYPE, typename ...MESSAGE_DISPATCHER_TYPES_REST>
   MessageTypeAdapter* GetMessage(meta::list<MESSAGE_DISPATCHER_TYPE, MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX msgNo) const 
   {
      constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<MESSAGE_DISPATCHER_TYPE>();
      if(msgNo < std::get<n>(msg_).size()) {
         auto it = std::get<n>(msg_).begin();
         for(INDEX i=0; i<msgNo; ++i) { ++it; }
         return *it;
         //return  std::get<n>(msg_)[msgNo]; 
      }
      else return GetMessage(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>{}, msgNo - std::get<n>(msg_).size());
   }
   MessageTypeAdapter* GetMessage(const INDEX n) const final
   {
      assert(n<no_messages());
      return GetMessage(MESSAGE_DISPATCHER_TYPELIST{}, n);
   }

   template<typename ...MESSAGE_DISPATCHER_TYPES_REST>
   FactorTypeAdapter* GetConnectedFactor(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX) const 
   {
      throw std::runtime_error("message index out of bound");
   }
   template<typename MESSAGE_DISPATCHER_TYPE, typename ...MESSAGE_DISPATCHER_TYPES_REST>
   FactorTypeAdapter* GetConnectedFactor(meta::list<MESSAGE_DISPATCHER_TYPE, MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX cur_msg_idx) const // to get the current message_type
   {
      constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<MESSAGE_DISPATCHER_TYPE>();
      const INDEX no_msgs = std::get<n>(msg_).size();
      if(cur_msg_idx < no_msgs) {
         //auto msg = std::get<n>(msg_)[cur_msg_idx];
         // do zrobienia: not most efficient way
         auto it = std::get<n>(msg_).begin();
         for(INDEX i=0; i<cur_msg_idx; ++i) { ++it; }
         auto msg = *it;
         assert(msg != nullptr);
         if(msg->GetLeftFactor() == static_cast<const FactorTypeAdapter*>(this)) { return msg->GetRightFactor(); }
         else { return msg->GetLeftFactor(); }
      } else {
         return GetConnectedFactor(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>{}, cur_msg_idx - no_msgs);
      }
   }
   FactorTypeAdapter* GetConnectedFactor (const INDEX msg_idx) const final
   { 
      auto f = GetConnectedFactor(MESSAGE_DISPATCHER_TYPELIST{}, msg_idx);
      assert(f != this);
      return f;
   }

   bool ReceivesMessage() const
   {
      bool can_receive = false;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [this,&can_receive](auto l) {
         constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
         if(l.ReceivesMessage() && std::get<n>(msg_).size() > 0) {
            can_receive = true;
         }
      });
      return can_receive;
   }

   // check whether actually send messages is called. Can be false, even if CanSendMessages is true, e.g. when no message is present
   bool SendsMessage() const
   {
      bool calls_send = false;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [&](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            if(l.SendsMessage() && std::get<n>(msg_).size() > 0) {
               calls_send = true;
            }
      });
      return calls_send;
   }


   template<typename ...MESSAGE_DISPATCHER_TYPES_REST>
   bool SendsMessage(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX) const 
   {
      throw std::runtime_error("message index out of bound");
   }
   template<typename MESSAGE_DISPATCHER_TYPE, typename ...MESSAGE_DISPATCHER_TYPES_REST>
   bool SendsMessage(meta::list<MESSAGE_DISPATCHER_TYPE, MESSAGE_DISPATCHER_TYPES_REST...>, const INDEX cur_msg_idx) const // to get the current MESSAGE_TYPE
   {
      constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<MESSAGE_DISPATCHER_TYPE>();
      const INDEX no_msgs = std::get<n>(msg_).size();
      if(cur_msg_idx < no_msgs) {
         MESSAGE_DISPATCHER_TYPE l;
         assert(std::get<n>(msg_).size() > 0);
         if( l.SendsMessage() ) {
            return true;
         } else {
           return false;
         }
      } else {
         return SendsMessage(meta::list<MESSAGE_DISPATCHER_TYPES_REST...>{}, cur_msg_idx - no_msgs);
      }
   }

   bool SendsMessage(const INDEX msg_idx) const final
   {
      return SendsMessage(MESSAGE_DISPATCHER_TYPELIST{}, msg_idx);
   }

   // check whether actually receive restricted messages is called. Can be false, even if CanReceiveRestrictedMessages is true, e.g. when no message is present
   bool ReceivesRestrictedMessage() const
   {
      bool calls_receive_restricted = false;
      meta::for_each(MESSAGE_DISPATCHER_TYPELIST{}, [&](auto l) {
            constexpr INDEX n = FactorContainerType::FindMessageDispatcherTypeIndex<decltype(l)>();
            if(l.CanCallReceiveRestrictedMessage() && std::get<n>(msg_).size() > 0) {
               calls_receive_restricted = true;
            }
      });
      return calls_receive_restricted;
   }

   // does factor call {Receive(Restricted)?|Send}Messages or does it compute primal? If not, UpdateFactor need not be called.
   bool FactorUpdated() const final
   {
      if(CanComputePrimal()) {
         return true;
      }
      if(ReceivesMessage()) {
         return true;
      }
      if(SendsMessage()) {
         return true;
      }
      if(ReceivesRestrictedMessage()) {
         return true;
      }
      return false;
   }

   void SetAndPropagatePrimal() const
   {
      assert(false);
     //assert(GetPrimalOffset() + PrimalSize() <= primal.size());
      //for(INDEX i=0; i<PrimalSize(); ++i) {
      //   primal[i + GetPrimalOffset()] = label[i];
      //}
      propagate_primal_through_messages();
   }

   struct apply_subgradient {
      apply_subgradient(double* _w, REAL _sign) : w(_w), sign(_sign) { assert(sign == 1.0 || sign == -1.0); }
      void operator[](const INDEX i) { w[i] = sign; }
      private:
      REAL* const w;
      const REAL sign;
   };
   constexpr static bool can_apply()
   {
      return FunctionExistence::has_apply<FactorType, void, apply_subgradient>(); 
   }
   virtual INDEX subgradient(double* w, const REAL sign) final
   {
      assert(sign == -1.0 || sign == 1.0);
      if constexpr(can_apply()) {
        apply_subgradient a(w,sign);
        factor_.apply(a);
      } else {
        assert(false);
      }
      return 0;
   }

   virtual REAL dot_product(double* w) final
   {
      class apply_dot_product {
      public:
         apply_dot_product(double* _w) : w(_w) {}
         void operator[](const INDEX i) { dp += w[i]; }
         REAL dot_product() const { return dp; }
      private:
         REAL* const w;
         REAL dp = 0;
      };

      apply_dot_product d(w);
      static_if<can_apply()>([this,w,&d](auto f) {
            f(factor_).apply(d);
      }).else_([](auto f) {
         assert(false);
      });
      return d.dot_product(); 
   }

   virtual void serialize_dual(load_archive& ar) final
   { factor_.serialize_dual(ar); }
   virtual void serialize_primal(load_archive& ar) final
   { factor_.serialize_primal(ar); } 
   virtual void serialize_dual(save_archive& ar) final
   { factor_.serialize_dual(ar); }
   virtual void serialize_primal(save_archive& ar) final
   { factor_.serialize_primal(ar); } 
   virtual void serialize_dual(allocate_archive& ar) final
   { factor_.serialize_dual(ar); }
   virtual void serialize_primal(allocate_archive& ar) final
   { factor_.serialize_primal(ar); } 
   virtual void serialize_dual(addition_archive& ar) final
   { factor_.serialize_dual(ar); }

   // returns size in bytes
   virtual INDEX dual_size() final
   {
      return dual_size_in_bytes()/sizeof(REAL);
   }

   virtual INDEX dual_size_in_bytes() final
   {
      allocate_archive ar;
      factor_.serialize_dual(ar);
      assert(ar.size() % sizeof(REAL) == 0);
      return ar.size();
   }

   virtual void divide(const REAL val) final
   {
      arithmetic_archive<operation::division> ar(val);
      factor_.serialize_dual(ar);
   }

   virtual INDEX primal_size_in_bytes() final
   {
      allocate_archive ar;
      factor_.serialize_primal(ar);
      return ar.size(); 
   }

   REAL LowerBound() const final {
      //return factor_.LowerBound(*this); 
      return factor_.LowerBound(); 
   } 

   FactorType* GetFactor() const { return &factor_; }
   FactorType* GetFactor() { return &factor_; }

  template<typename MESSAGE_TYPE>
  constexpr static 
  INDEX get_message_number()
  {
     static_assert(MESSAGE_TYPE::leftFactorNumber == FACTOR_NO || MESSAGE_TYPE::rightFactorNumber == FACTOR_NO,"");
     static_assert(MESSAGE_TYPE::leftFactorNumber != MESSAGE_TYPE::rightFactorNumber,""); // otherwise we cannot distinguish

     constexpr bool left = MESSAGE_TYPE::leftFactorNumber == FACTOR_NO;
     using dispatcher_type = typename meta::if_c<left, MessageDispatcher<MESSAGE_TYPE, LeftMessageFuncGetter> , MessageDispatcher<MESSAGE_TYPE, RightMessageFuncGetter>>;
     return  FactorContainerType::FindMessageDispatcherTypeIndex<dispatcher_type>();

     //if(MESSAGE_TYPE::leftFactorNumber == FACTOR_NO) {
     //   return FindMessageDispatcherTypeIndex<MessageDispatcher<MESSAGE_TYPE, LeftMessageFuncGetter>>();
     //} else {
     //   return FindMessageDispatcherTypeIndex<MessageDispatcher<MESSAGE_TYPE, RightMessageFuncGetter>>();
     //} 
  }

  template<typename MESSAGE_TYPE>
  auto get_messages() const 
  {
     return std::get< get_message_number<MESSAGE_TYPE>() >(msg_);
  }
   
protected:
   FactorType factor_; // the factor operation
public:
   INDEX primal_access_ = 0; // counts when primal was accessed last, do zrobienia: make setter and getter for clean interface or make MessageContainer a friend

   virtual void init_primal() final
   {
      factor_.init_primal();
   }
   void conditionally_init_primal(const INDEX timestamp) 
   {
      assert(primal_access_ <= timestamp);
      if(primal_access_ < timestamp) {
         factor_.init_primal();
         primal_access_ = timestamp;
      } 
   }

   INDEX runtime_estimate()
   {
     INDEX runtime = 0;
     assert(false);
     // go over all messages to be received and sum the dual size of connected factors

     // get number of messages to be sent and multiply by dual size of current factor (discount for SendMessages calls?)

     return runtime;
   }
protected:
   // pool memory allocator specific for this factor container
   // note: the below construction is not perfect when more than one solver is run simultaneously: The same allocator is used, yet the optimization problems are different + not thread safe.
   // -> investigate thread_local inline static! inline static however is only supported in C++17
   struct Allocator { // we enclose static allocator in nested class as only there (since C++11) we can access sizeof(FactorContainerType).
      using type = MemoryPool<FactorContainerType,4096*(sizeof(FactorContainerType)+sizeof(void*))>; 
      static type& get() {
         static type allocator;
         return allocator;
      }
   };
   
   // compile time metaprogramming to transform Factor-Message information into lists of which messages this factor must hold
   // first get lists with left and right message types
   struct get_msg_type_list {
      template<typename LIST>
         using invoke = typename LIST::MessageContainerType;
   };
   struct get_left_msg {
      template<class LIST>
         using invoke = typename std::is_same<meta::size_t<LIST::leftFactorNumber>, meta::size_t<FACTOR_NO>>::type;
   };
   struct get_right_msg {
      template<class LIST>
         using invoke = typename std::is_same<meta::size_t<LIST::rightFactorNumber>, meta::size_t<FACTOR_NO> >::type;
   };
   struct get_left_msg_container_type_list {
      template<class LIST>
         using invoke = typename LIST::LeftMessageContainerStorageType;
   };
   struct get_right_msg_container_type_list {
      template<class LIST>
         using invoke = typename LIST::RightMessageContainerStorageType;
   };

   using left_msg_list = meta::transform< meta::filter<typename FACTOR_MESSAGE_TRAIT::MessageList, get_left_msg>, get_msg_type_list>;
   using right_msg_list = meta::transform< meta::filter<typename FACTOR_MESSAGE_TRAIT::MessageList, get_right_msg>, get_msg_type_list>;
   using left_msg_container_list = meta::transform< meta::filter<typename FACTOR_MESSAGE_TRAIT::MessageList, get_left_msg>, get_left_msg_container_type_list>;
   using right_msg_container_list = meta::transform< meta::filter<typename FACTOR_MESSAGE_TRAIT::MessageList, get_right_msg>, get_right_msg_container_type_list>;

   // now construct a tuple with left and right dispatcher
   struct left_dispatch {
      template<class LIST>
         using invoke = MessageDispatcher<LIST, LeftMessageFuncGetter>;
   };
   struct right_dispatch {
      template<class LIST>
         using invoke = MessageDispatcher<LIST, RightMessageFuncGetter>;
   };
   using left_dispatcher_list = meta::transform< left_msg_list, left_dispatch >;
   using right_dispatcher_list = meta::transform< right_msg_list, right_dispatch >;

   using MESSAGE_DISPATCHER_TYPELIST = meta::concat<left_dispatcher_list, right_dispatcher_list>;

public:
   // helper function for getting the index in msg_ of given MESSAGE_DISPATCHER_TYPE
   template<typename MESSAGE_DISPATCHER_TYPE>
   static constexpr INDEX FindMessageDispatcherTypeIndex()
   {
      constexpr INDEX n = meta::find_index<MESSAGE_DISPATCHER_TYPELIST, MESSAGE_DISPATCHER_TYPE>::value;
      static_assert(n < meta::size<MESSAGE_DISPATCHER_TYPELIST>::value,"");
      return n;
   }


   // construct tuple holding messages for left and right dispatch
   // the tuple will hold some container for the message type. The container type is specified in the {Left|Right}MessageContainerStorageType fields of MessageList
   using msg_container_type_list = meta::concat<left_msg_container_list, right_msg_container_list>;
private:

   tuple_from_list<msg_container_type_list> msg_;

public:
   REAL EvaluatePrimal() const final
   {
      //return factor_.EvaluatePrimal(*this,primalIt + primalOffset_);
      //return factor_.EvaluatePrimal(primalIt + primalOffset_);
      return factor_.EvaluatePrimal();
   }

#ifdef LP_MP_PARALLEL
   // a recursive mutex is required only for SendMessagesTo{Left|Right}, as multiple messages may be have the same endpoints. Then the corresponding lock is acquired multiple times.
   // if no two messages have the same endpoints, an ordinary mutex is enough.
   std::recursive_mutex mutex_;
#endif

   // functions for interfacing with external solver interface DD_ILP

   template<typename EXTERNAL_SOLVER>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, REAL x)
   { return s.add_variable(); }

   template<typename EXTERNAL_SOLVER>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, const vector<REAL>& v)
   { return s.add_vector(v); }

   template<typename EXTERNAL_SOLVER, std::size_t N>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, const std::array<REAL,N>& v)
   { return s.add_vector(v); }

   template<typename EXTERNAL_SOLVER>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, const std::vector<REAL>& v)
   { return s.add_vector(v); } 

   template<typename EXTERNAL_SOLVER>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, const matrix<REAL>& m)
   { return s.add_matrix(m); }

   template<typename EXTERNAL_SOLVER>
   auto convert_variables_to_external(EXTERNAL_SOLVER& s, const tensor3<REAL>& t)
   { return s.add_tensor(t); }

   template<typename EXTERNAL_SOLVER>
   auto load_external_variables(EXTERNAL_SOLVER& s, REAL x)
   { return s.load_variable(); }

   template<typename EXTERNAL_SOLVER>
   auto load_external_variables(EXTERNAL_SOLVER& s, vector<REAL>& x)
   { return s.load_vector(); }

   template<typename EXTERNAL_SOLVER, INDEX N>
   auto load_external_variables(EXTERNAL_SOLVER& s, array<REAL,N>& x)
   { return s.load_vector(); }

   template<typename EXTERNAL_SOLVER>
   auto load_external_variables(EXTERNAL_SOLVER& s, std::vector<REAL>& x)
   { return s.load_vector(); }

   template<typename EXTERNAL_SOLVER>
   auto load_external_variables(EXTERNAL_SOLVER& s, matrix<REAL>& x)
   { return s.load_matrix(); }

   template<typename EXTERNAL_SOLVER>
   auto load_external_variables(EXTERNAL_SOLVER& s, tensor3<REAL>& x)
   { return s.load_tensor(); }

   template<typename EXTERNAL_SOLVER>
   void add_objective(EXTERNAL_SOLVER& s, REAL cost)
   { s.add_variable_objective(cost); }

   template<typename EXTERNAL_SOLVER>
   void add_objective(EXTERNAL_SOLVER& s, const vector<REAL>& cost)
   { s.add_vector_objective(cost); }

   template<typename EXTERNAL_SOLVER, std::size_t N>
   auto add_objective(EXTERNAL_SOLVER& s, array<REAL,N>& cost)
   { return s.add_vector_objective(cost); } 

   template<typename EXTERNAL_SOLVER>
   void add_objective(EXTERNAL_SOLVER& s, const std::vector<REAL>& cost)
   { s.add_vector_objective(cost); }

   template<typename EXTERNAL_SOLVER>
   void add_objective(EXTERNAL_SOLVER& s, const matrix<REAL>& cost)
   { s.add_matrix_objective(cost); }

   template<typename EXTERNAL_SOLVER>
   void add_objective(EXTERNAL_SOLVER& s, const tensor3<REAL>& cost)
   { s.add_tensor_objective(cost); }

   // functions for implementing external solver interface
   template<typename EXTERNAL_SOLVER>
   void construct_constraints_impl(EXTERNAL_SOLVER& s)
   {
      // transform exported variables to external solver variables
      auto vars = factor_.export_variables();
      auto external_vars = std::apply([this,&s](auto... x){ return std::make_tuple(this->convert_variables_to_external(s, x)...); }, vars);

      // unpack tuple and call construct_constraints function of factor
      auto construct_constraints_fun = [this,&s](auto... x) { this->factor_.construct_constraints(s, x...); };
      std::apply(construct_constraints_fun, external_vars);
   }


   template<typename EXTERNAL_SOLVER>
   void load_costs_impl(EXTERNAL_SOLVER& s)
   {
      // FIXME: This code relies on UNDEFINED BEHAVIOUR!
      // The order of argument evaluation for function calls is undefined
      // behaviour even in C++17! At the same time, the side_effects of
      // `add_objective` and `convert_primal` must take effect in exactly the
      // same order.

      // load external solver variables corresponding to reparametrization ones and add reparametrization as cost
      auto vars = factor_.export_variables();
      std::apply([this,&s](auto... x){ std::make_tuple((this->add_objective(s,x), 0)...); }, vars);
      //auto external_vars = std::apply([this,&s](auto... x){ return std::make_tuple(this->leftFactor_->load_external_variables(s, x)...); }, vars);
      // for all variables,
   }


   template<typename SOLVER>
   void convert_primal_impl(SOLVER& s)
   {
      // FIXME: This code relies on UNDEFINED BEHAVIOUR!
      // See explanation in function `load_costs_impl`.

      auto vars = factor_.export_variables();
      auto external_vars = std::apply([this,&s](auto... x){ return std::make_tuple(this->load_external_variables(s, x)...); }, vars); 

      auto convert_primal_fun = [this,&s](auto... x) { this->factor_.convert_primal(s, x...); };
      std::apply(convert_primal_fun, external_vars);

      //propagate_primal_through_messages();
   }

   virtual void construct_constraints(DD_ILP::external_solver_interface<DD_ILP::sat_solver>& s) final { construct_constraints_impl(s); }
   virtual void load_costs(DD_ILP::external_solver_interface<DD_ILP::sat_solver>& s) final {  }
   virtual void convert_primal(DD_ILP::external_solver_interface<DD_ILP::sat_solver>& solver) { convert_primal_impl(solver); }

   virtual void construct_constraints(DD_ILP::external_solver_interface<DD_ILP::problem_export>& s) final { construct_constraints_impl(s); }
   virtual void load_costs(DD_ILP::external_solver_interface<DD_ILP::problem_export>& s) final { load_costs_impl(s); } 
   virtual void convert_primal(DD_ILP::external_solver_interface<DD_ILP::problem_export>& solver) { convert_primal_impl(solver); }

#ifdef DD_ILP_WITH_GUROBI
   virtual void construct_constraints(DD_ILP::external_solver_interface<DD_ILP::gurobi_interface>& s) final { construct_constraints_impl(s); }
   virtual void load_costs(DD_ILP::external_solver_interface<DD_ILP::gurobi_interface>& s) final { load_costs_impl(s); } 
   virtual void convert_primal(DD_ILP::external_solver_interface<DD_ILP::gurobi_interface>& solver) { convert_primal_impl(solver); }
#endif
   /*
   // sat related functions
   LP_MP_FUNCTION_EXISTENCE_CLASS(has_construct_constraints,construct_constraints)

   constexpr static bool
   can_construct_constraints()
   {
      return has_construct_constraints<FactorType,void,sat_solver&>();
   }

   void construct_sat_clauses(sat_solver& s) 
   {
      static_if<can_construct_sat_clauses()>([&](auto f) {
            f(factor_).construct_sat_clauses(s);
            });
      if(!can_construct_sat_clauses()) {
         assert(false);
      }
   }


   template<typename SAT_SOLVER>
   constexpr static bool can_convert_primal()
   {
      return FunctionExistence::has_convert_primal<FactorType,void, SAT_SOLVER, sat_var>(); 
   }
   //void convert_primal(Glucose::SimpSolver& sat, const sat_var sat_begin) final // this is not nice: the solver should be templatized
   //void convert_primal(CMSat::SATSolver& sat, const sat_var sat_begin) final // this is not nice: the solver should be templatized
   void convert_primal(sat_solver& sat, const sat_var sat_begin) final // this is not nice: the solver should be templatized
   {
      static_if<can_convert_primal<decltype(sat)>()>([&](auto f) { 
            f(factor_).convert_primal(sat, sat_begin);
            });
      assert(can_convert_primal<decltype(sat)>);
   }

   void reduce_sat(const REAL th, sat_var begin, std::vector<sat_literal>& assumptions) final
   {
     static_if<can_reduce_sat()>([&](auto f) {
       f(factor_).reduce_sat(assumptions, th, begin);
     }).else_([&](auto) {
       assert(false);  
     }); 
   }
   */

};

} // end namespace LP_MP

#endif // LP_MP_FACTORS_MESSAGES_HXX

