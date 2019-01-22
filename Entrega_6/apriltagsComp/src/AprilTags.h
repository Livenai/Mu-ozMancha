// **********************************************************************
//
// Copyright (c) 2003-2017 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.7.0
//
// <auto-generated>
//
// Generated from file `AprilTags.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __AprilTags_h__
#define __AprilTags_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <JointMotor.h>
#include <GenericBase.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompAprilTags
{

class AprilTags;
class AprilTagsPrx;

}

namespace RoboCompAprilTags
{

struct tag
{
    int id;
    float tx;
    float ty;
    float tz;
    float rx;
    float ry;
    float rz;
    ::std::string cameraId;

    std::tuple<const int&, const float&, const float&, const float&, const float&, const float&, const float&, const ::std::string&> ice_tuple() const
    {
        return std::tie(id, tx, ty, tz, rx, ry, rz, cameraId);
    }
};

using tagsList = ::std::vector<::RoboCompAprilTags::tag>;

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompAprilTags
{

class AprilTags : public virtual ::Ice::Object
{
public:

    using ProxyType = AprilTagsPrx;

    virtual bool ice_isA(::std::string, const ::Ice::Current&) const override;
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current&) const override;
    virtual ::std::string ice_id(const ::Ice::Current&) const override;

    static const ::std::string& ice_staticId();

    virtual void newAprilTag(::RoboCompAprilTags::tagsList, const ::Ice::Current&) = 0;
    bool _iceD_newAprilTag(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void newAprilTagAndPose(::RoboCompAprilTags::tagsList, ::RoboCompGenericBase::TBaseState, ::RoboCompJointMotor::MotorStateMap, const ::Ice::Current&) = 0;
    bool _iceD_newAprilTagAndPose(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
};

}

namespace RoboCompAprilTags
{

class AprilTagsPrx : public virtual ::Ice::Proxy<AprilTagsPrx, ::Ice::ObjectPrx>
{
public:

    void newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context = Ice::noExplicitContext)
    {
        _makePromiseOutgoing<void>(true, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTag, iceP_tags, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto newAprilTagAsync(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context = Ice::noExplicitContext)
        -> decltype(::std::declval<P<void>>().get_future())
    {
        return _makePromiseOutgoing<void, P>(false, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTag, iceP_tags, context);
    }

    ::std::function<void()>
    newAprilTagAsync(const ::RoboCompAprilTags::tagsList& iceP_tags,
                     ::std::function<void()> response,
                     ::std::function<void(::std::exception_ptr)> ex = nullptr,
                     ::std::function<void(bool)> sent = nullptr,
                     const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<void>(response, ex, sent, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTag, iceP_tags, context);
    }

    void _iceI_newAprilTag(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<void>>&, const ::RoboCompAprilTags::tagsList&, const ::Ice::Context&);

    void newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context = Ice::noExplicitContext)
    {
        _makePromiseOutgoing<void>(true, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTagAndPose, iceP_tags, iceP_bState, iceP_hState, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto newAprilTagAndPoseAsync(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context = Ice::noExplicitContext)
        -> decltype(::std::declval<P<void>>().get_future())
    {
        return _makePromiseOutgoing<void, P>(false, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTagAndPose, iceP_tags, iceP_bState, iceP_hState, context);
    }

    ::std::function<void()>
    newAprilTagAndPoseAsync(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState,
                            ::std::function<void()> response,
                            ::std::function<void(::std::exception_ptr)> ex = nullptr,
                            ::std::function<void(bool)> sent = nullptr,
                            const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<void>(response, ex, sent, this, &RoboCompAprilTags::AprilTagsPrx::_iceI_newAprilTagAndPose, iceP_tags, iceP_bState, iceP_hState, context);
    }

    void _iceI_newAprilTagAndPose(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<void>>&, const ::RoboCompAprilTags::tagsList&, const ::RoboCompGenericBase::TBaseState&, const ::RoboCompJointMotor::MotorStateMap&, const ::Ice::Context&);

    static const ::std::string& ice_staticId();

protected:

    AprilTagsPrx() = default;
    friend ::std::shared_ptr<AprilTagsPrx> IceInternal::createProxy<AprilTagsPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
};

}

namespace Ice
{

template<>
struct StreamableTraits<::RoboCompAprilTags::tag>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 29;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompAprilTags::tag, S>
{
    static void read(S* istr, ::RoboCompAprilTags::tag& v)
    {
        istr->readAll(v.id, v.tx, v.ty, v.tz, v.rx, v.ry, v.rz, v.cameraId);
    }
};

}

namespace RoboCompAprilTags
{

using AprilTagsPtr = ::std::shared_ptr<AprilTags>;
using AprilTagsPrxPtr = ::std::shared_ptr<AprilTagsPrx>;

}

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompAprilTags
{

class AprilTags;
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompAprilTags::AprilTags>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompAprilTags::AprilTags*);

}

}

namespace RoboCompAprilTags
{

class AprilTags;
::Ice::Object* upCast(::RoboCompAprilTags::AprilTags*);
typedef ::IceInternal::Handle< ::RoboCompAprilTags::AprilTags> AprilTagsPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompAprilTags::AprilTags> AprilTagsPrx;
typedef AprilTagsPrx AprilTagsPrxPtr;
void _icePatchObjectPtr(AprilTagsPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompAprilTags
{

struct tag
{
    ::Ice::Int id;
    ::Ice::Float tx;
    ::Ice::Float ty;
    ::Ice::Float tz;
    ::Ice::Float rx;
    ::Ice::Float ry;
    ::Ice::Float rz;
    ::std::string cameraId;

    bool operator==(const tag& rhs_) const
    {
        if(this == &rhs_)
        {
            return true;
        }
        if(id != rhs_.id)
        {
            return false;
        }
        if(tx != rhs_.tx)
        {
            return false;
        }
        if(ty != rhs_.ty)
        {
            return false;
        }
        if(tz != rhs_.tz)
        {
            return false;
        }
        if(rx != rhs_.rx)
        {
            return false;
        }
        if(ry != rhs_.ry)
        {
            return false;
        }
        if(rz != rhs_.rz)
        {
            return false;
        }
        if(cameraId != rhs_.cameraId)
        {
            return false;
        }
        return true;
    }

    bool operator<(const tag& rhs_) const
    {
        if(this == &rhs_)
        {
            return false;
        }
        if(id < rhs_.id)
        {
            return true;
        }
        else if(rhs_.id < id)
        {
            return false;
        }
        if(tx < rhs_.tx)
        {
            return true;
        }
        else if(rhs_.tx < tx)
        {
            return false;
        }
        if(ty < rhs_.ty)
        {
            return true;
        }
        else if(rhs_.ty < ty)
        {
            return false;
        }
        if(tz < rhs_.tz)
        {
            return true;
        }
        else if(rhs_.tz < tz)
        {
            return false;
        }
        if(rx < rhs_.rx)
        {
            return true;
        }
        else if(rhs_.rx < rx)
        {
            return false;
        }
        if(ry < rhs_.ry)
        {
            return true;
        }
        else if(rhs_.ry < ry)
        {
            return false;
        }
        if(rz < rhs_.rz)
        {
            return true;
        }
        else if(rhs_.rz < rz)
        {
            return false;
        }
        if(cameraId < rhs_.cameraId)
        {
            return true;
        }
        else if(rhs_.cameraId < cameraId)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const tag& rhs_) const
    {
        return !operator==(rhs_);
    }
    bool operator<=(const tag& rhs_) const
    {
        return operator<(rhs_) || operator==(rhs_);
    }
    bool operator>(const tag& rhs_) const
    {
        return !operator<(rhs_) && !operator==(rhs_);
    }
    bool operator>=(const tag& rhs_) const
    {
        return !operator<(rhs_);
    }
};

typedef ::std::vector< ::RoboCompAprilTags::tag> tagsList;

}

namespace RoboCompAprilTags
{

class Callback_AprilTags_newAprilTag_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_AprilTags_newAprilTag_Base> Callback_AprilTags_newAprilTagPtr;

class Callback_AprilTags_newAprilTagAndPose_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_AprilTags_newAprilTagAndPose_Base> Callback_AprilTags_newAprilTagAndPosePtr;

}

namespace IceProxy
{

namespace RoboCompAprilTags
{

class AprilTags : public virtual ::Ice::Proxy<AprilTags, ::IceProxy::Ice::Object>
{
public:

    void newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        end_newAprilTag(_iceI_begin_newAprilTag(iceP_tags, context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_newAprilTag(iceP_tags, context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTag(iceP_tags, ::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTag(iceP_tags, context, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompAprilTags::Callback_AprilTags_newAprilTagPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTag(iceP_tags, ::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTag(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::Ice::Context& context, const ::RoboCompAprilTags::Callback_AprilTags_newAprilTagPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTag(iceP_tags, context, del, cookie);
    }

    void end_newAprilTag(const ::Ice::AsyncResultPtr&);

private:

    ::Ice::AsyncResultPtr _iceI_begin_newAprilTag(const ::RoboCompAprilTags::tagsList&, const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    void newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        end_newAprilTagAndPose(_iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, ::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, context, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::RoboCompAprilTags::Callback_AprilTags_newAprilTagAndPosePtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, ::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList& iceP_tags, const ::RoboCompGenericBase::TBaseState& iceP_bState, const ::RoboCompJointMotor::MotorStateMap& iceP_hState, const ::Ice::Context& context, const ::RoboCompAprilTags::Callback_AprilTags_newAprilTagAndPosePtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_newAprilTagAndPose(iceP_tags, iceP_bState, iceP_hState, context, del, cookie);
    }

    void end_newAprilTagAndPose(const ::Ice::AsyncResultPtr&);

private:

    ::Ice::AsyncResultPtr _iceI_begin_newAprilTagAndPose(const ::RoboCompAprilTags::tagsList&, const ::RoboCompGenericBase::TBaseState&, const ::RoboCompJointMotor::MotorStateMap&, const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    static const ::std::string& ice_staticId();

protected:

    virtual ::IceProxy::Ice::Object* _newInstance() const;
};

}

}

namespace RoboCompAprilTags
{

class AprilTags : public virtual ::Ice::Object
{
public:

    typedef AprilTagsPrx ProxyType;
    typedef AprilTagsPtr PointerType;

    virtual ~AprilTags();

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::emptyCurrent) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::emptyCurrent) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::emptyCurrent) const;

    static const ::std::string& ice_staticId();

    virtual void newAprilTag(const ::RoboCompAprilTags::tagsList&, const ::Ice::Current& = ::Ice::emptyCurrent) = 0;
    bool _iceD_newAprilTag(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void newAprilTagAndPose(const ::RoboCompAprilTags::tagsList&, const ::RoboCompGenericBase::TBaseState&, const ::RoboCompJointMotor::MotorStateMap&, const ::Ice::Current& = ::Ice::emptyCurrent) = 0;
    bool _iceD_newAprilTagAndPose(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:

    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
};

inline bool operator==(const AprilTags& lhs, const AprilTags& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const AprilTags& lhs, const AprilTags& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}

}

namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompAprilTags::tag>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 29;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompAprilTags::tag, S>
{
    static void write(S* ostr, const ::RoboCompAprilTags::tag& v)
    {
        ostr->write(v.id);
        ostr->write(v.tx);
        ostr->write(v.ty);
        ostr->write(v.tz);
        ostr->write(v.rx);
        ostr->write(v.ry);
        ostr->write(v.rz);
        ostr->write(v.cameraId);
    }
};

template<typename S>
struct StreamReader< ::RoboCompAprilTags::tag, S>
{
    static void read(S* istr, ::RoboCompAprilTags::tag& v)
    {
        istr->read(v.id);
        istr->read(v.tx);
        istr->read(v.ty);
        istr->read(v.tz);
        istr->read(v.rx);
        istr->read(v.ry);
        istr->read(v.rz);
        istr->read(v.cameraId);
    }
};

}

namespace RoboCompAprilTags
{

template<class T>
class CallbackNC_AprilTags_newAprilTag : public Callback_AprilTags_newAprilTag_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_AprilTags_newAprilTag(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

template<class T> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTag<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTag<T>(instance, 0, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTag<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTag<T>(instance, 0, excb, sentcb);
}

template<class T, typename CT>
class Callback_AprilTags_newAprilTag : public Callback_AprilTags_newAprilTag_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_AprilTags_newAprilTag(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

template<class T, typename CT> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTag<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTag<T, CT>(instance, 0, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTag<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagPtr
newCallback_AprilTags_newAprilTag(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTag<T, CT>(instance, 0, excb, sentcb);
}

template<class T>
class CallbackNC_AprilTags_newAprilTagAndPose : public Callback_AprilTags_newAprilTagAndPose_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_AprilTags_newAprilTagAndPose(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

template<class T> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTagAndPose<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTagAndPose<T>(instance, 0, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTagAndPose<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_AprilTags_newAprilTagAndPose<T>(instance, 0, excb, sentcb);
}

template<class T, typename CT>
class Callback_AprilTags_newAprilTagAndPose : public Callback_AprilTags_newAprilTagAndPose_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_AprilTags_newAprilTagAndPose(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

template<class T, typename CT> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTagAndPose<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTagAndPose<T, CT>(instance, 0, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTagAndPose<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_AprilTags_newAprilTagAndPosePtr
newCallback_AprilTags_newAprilTagAndPose(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_AprilTags_newAprilTagAndPose<T, CT>(instance, 0, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif
