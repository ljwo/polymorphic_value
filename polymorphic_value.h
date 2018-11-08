/*

Copyright (c) 2016 Jonathan B. Coe

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef JBCOE_POLYMORPHIC_VALUE_H_INCLUDED
#define JBCOE_POLYMORPHIC_VALUE_H_INCLUDED

#include <cassert>
#include <exception>
#include <memory>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <functional>

namespace jbcoe
{

  namespace detail
  {

    ////////////////////////////////////////////////////////////////////////////
    // Implementation detail classes
    ////////////////////////////////////////////////////////////////////////////

    template <class T>
    struct default_copy
    {
      T* operator()(const T& t) const
      {
        return new T(t);
      }
    };

    template <class T>
    struct default_delete
    {
      void operator()(const T* t) const
      {
        delete t;
      }
    };

    struct Nothing
    {
      template<class T>
      struct Base {};
      template<class CBD>
      struct Custom : public CBD {};
      template<class Poly>
      struct Immediate {};
    };

    struct Hashable;

//    struct Hashable
//    {
//      template<class T>
//      struct Base
//      {
//        virtual std::size_t get_hash() = 0;
//      };
//      template<class CBD>
//      struct Custom : public CBD
//      {
//        std::size_t get_hash() override
//        {
//        return std::hash<std::decay_t<decltype(*CBD::derived_ptr())>>()(*(this->derived_ptr()));
//        }
//      };
//      template<class Poly>
//      struct Immediate
//      {
//        std::size_t get_hash() const
//        {
//        return static_cast<const Poly*>(this)->cb_->get_hash();
//        }
//      };
//    };


    template <class T, class Policy>
    struct control_block : public Policy::template Base<T>
    {
      virtual ~control_block() = default;

      virtual std::unique_ptr<control_block> clone() const = 0;

      virtual T* ptr() = 0;
    };

    template<class T, class U, class Policy>
    struct control_block_derived : public control_block<T,Policy>
    {
      using BaseT = T;
      virtual U* derived_ptr() = 0;
    };

    template<class T, class U, class Policy>
    struct control_block_custom : public Policy::template Custom<control_block_derived<T,U,Policy>>
    {};

    template <class T, class U = T, class Policy = Nothing>
    class direct_control_block : public control_block_custom<T,U,Policy>
    {
      static_assert(!std::is_reference<U>::value, "");
      U u_;

    public:
      template <class... Ts>
      explicit direct_control_block(Ts&&... ts) : u_(U(std::forward<Ts>(ts)...))
      {
      }

      std::unique_ptr<control_block<T,Policy>> clone() const override
      {
        return std::make_unique<direct_control_block>(*this);
      }

      T* ptr() override
      {
        return &u_;
      }
      U* derived_ptr() override
      {
        return &u_;
      }
    };

    template <class T, class U, class Policy = Nothing, class C = default_copy<U>,
              class D = default_delete<U>>
    class pointer_control_block : public control_block_custom<T,U, Policy>, public C
    {
      std::unique_ptr<U, D> p_;

    public:
      explicit pointer_control_block(U* u, C c = C{}, D d = D{})
          : C(std::move(c)), p_(u, std::move(d))
      {
      }

      explicit pointer_control_block(std::unique_ptr<U, D> p, C c = C{})
          : C(std::move(c)), p_(std::move(p))
      {
      }

      std::unique_ptr<control_block<T,Policy>> clone() const override
      {
        assert(p_);
        return std::make_unique<pointer_control_block>(
            C::operator()(*p_), static_cast<const C&>(*this), p_.get_deleter());
      }

      T* ptr() override
      {
        return p_.get();
      }
      U* derived_ptr() override
      {
        return p_.get();
      }
    };

    template <class T, class U, class Policy = Nothing>
    class delegating_control_block : public control_block_custom<T,U,Policy>
    {
      std::unique_ptr<control_block<U,Policy>> delegate_;

    public:
      explicit delegating_control_block(std::unique_ptr<control_block<U,Policy>> b)
          : delegate_(std::move(b))
      {
      }

      std::unique_ptr<control_block<T,Policy>> clone() const override
      {
        return std::make_unique<delegating_control_block>(delegate_->clone());
      }

      T* ptr() override
      {
        return delegate_->ptr();
      }
      U* derived_ptr() override
      {
        return delegate_->ptr();
      }
    };

    //struct Nothing {};

  } // end namespace detail

  class bad_polymorphic_value_construction : std::exception
  {
  public:
    bad_polymorphic_value_construction() noexcept = default;

    const char* what() const noexcept override
    {
      return "Dynamic and static type mismatch in polymorphic_value "
             "construction";
    }
  };

  template <class T, class Policy>
  class polymorphic_value;

  template <class T>
  struct is_polymorphic_value : std::false_type
  {
  };

  template <class T, class Policy>
  struct is_polymorphic_value<polymorphic_value<T,Policy>> : std::true_type
  {
  };


  ////////////////////////////////////////////////////////////////////////////////
  // `polymorphic_value` class definition
  ////////////////////////////////////////////////////////////////////////////////

  template <class T, class Policy = detail::Nothing>
  class polymorphic_value : public Policy::template Immediate<polymorphic_value<T,Policy>>
  {
    static_assert(!std::is_union<T>::value, "");
    static_assert(std::is_class<T>::value, "");

    template <class U, class UPolicy>
    friend class polymorphic_value;

    template <class T_, class U, class... Ts>
    friend polymorphic_value<T_,detail::Nothing> make_polymorphic_value(Ts&&... ts);
    template <class T_, class... Ts>
    friend polymorphic_value<T_,detail::Nothing> make_polymorphic_value(Ts&&... ts);

    T* ptr_ = nullptr;
  public:
  // ljw: Dirty hack for now, the Immediate should be on top of some polymorphic_impl
  // and then this could be protected
    std::unique_ptr<detail::control_block<T,Policy>> cb_;

  public:
    //
    // Destructor
    //

    ~polymorphic_value() = default;


    //
    // Constructors
    //

    polymorphic_value()
    {
    }

    template <class U, class C = detail::default_copy<U>,
              class D = detail::default_delete<U>,
              class V = std::enable_if_t<std::is_convertible<U*, T*>::value>>
    explicit polymorphic_value(U* u, C copier = C{}, D deleter = D{})
    {
      if (!u)
      {
        return;
      }

      if (std::is_same<D, detail::default_delete<U>>::value &&
          std::is_same<C, detail::default_copy<U>>::value &&
          typeid(*u) != typeid(U))
        throw bad_polymorphic_value_construction();
// ljw: Why throw and not fail to compile? Could do that if no pointer conversion?
      std::unique_ptr<U, D> p(u, std::move(deleter));

      cb_ = std::make_unique<detail::pointer_control_block<T, U, Policy, C, D>>(
          std::move(p), std::move(copier));
      ptr_ = u;
    }

  template <class... Ts>
  explicit polymorphic_value(std::in_place_type_t<T>, std::in_place_type_t<Policy>, Ts&&... ts)
  {
    cb_ = std::make_unique<detail::direct_control_block<T, T, Policy>>(
        std::forward<Ts>(ts)...);
    ptr_ = cb_->ptr();
  }


    //
    // Copy-constructors
    //

    polymorphic_value(const polymorphic_value& p)
    {
      if (!p)
      {
        return;
      }
      auto tmp_cb = p.cb_->clone();
      ptr_ = tmp_cb->ptr();
      cb_ = std::move(tmp_cb);
    }

    //
    // Move-constructors
    //

    polymorphic_value(polymorphic_value&& p) noexcept
    {
      ptr_ = p.ptr_;
      cb_ = std::move(p.cb_);
      p.ptr_ = nullptr;
    }

    //
    // Converting constructors
    //
// ljw: Why explicit?
    template <class U,
              class V = std::enable_if_t<!std::is_same<T, U>::value &&
                                         std::is_convertible<U*, T*>::value>>
    explicit polymorphic_value(const polymorphic_value<U, Policy>& p)
    {
      polymorphic_value<U, Policy> tmp(p);
      ptr_ = tmp.ptr_;
      cb_ = std::make_unique<detail::delegating_control_block<T, U, Policy>>(
          std::move(tmp.cb_));
    }

    template <class U,
              class V = std::enable_if_t<!std::is_same<T, U>::value &&
                                         std::is_convertible<U*, T*>::value>>
    explicit polymorphic_value(polymorphic_value<U>&& p)
    {
      ptr_ = p.ptr_;
      cb_ = std::make_unique<detail::delegating_control_block<T, U>>(
          std::move(p.cb_));
      p.ptr_ = nullptr;
    }

    //
    // Forwarding constructor
    //

    template <class U, class V = std::enable_if_t<
                           std::is_convertible<std::decay_t<U>*, T*>::value &&
                           !is_polymorphic_value<std::decay_t<U>>::value>>
    explicit polymorphic_value(U&& u)
        : cb_(std::make_unique<
              detail::direct_control_block<T, std::decay_t<U>, Policy>>(
              std::forward<U>(u)))
    {
      ptr_ = cb_->ptr();
    }


    //
    // Assignment
    //

    polymorphic_value& operator=(const polymorphic_value& p)
    {
      if (&p == this)
      {
        return *this;
      }

      if (!p)
      {
        cb_.reset();
        ptr_ = nullptr;
        return *this;
      }

      auto tmp_cb = p.cb_->clone();
      ptr_ = tmp_cb->ptr();
      cb_ = std::move(tmp_cb);
      return *this;
    }


    //
    // Move-assignment
    //

    polymorphic_value& operator=(polymorphic_value&& p) noexcept
    {
      if (&p == this)
      {
        return *this;
      }

      cb_ = std::move(p.cb_);
      ptr_ = p.ptr_;
      p.ptr_ = nullptr;
      return *this;
    }


    //
    // Modifiers
    //

    void swap(polymorphic_value& p) noexcept
    {
      using std::swap;
      swap(ptr_, p.ptr_);
      swap(cb_, p.cb_);
    }


    //
    // Observers
    //

    explicit operator bool() const
    {
      return (bool)cb_;
    }

    const T* operator->() const
    {
      assert(ptr_);
      return ptr_;
    }

    const T& operator*() const
    {
      assert(*this);
      return *ptr_;
    }

    T* operator->()
    {
      assert(*this);
      return ptr_;
    }

    T& operator*()
    {
      assert(*this);
      return *ptr_;
    }
  };

  template<class T, class Policy, class... Ts>
  polymorphic_value(std::in_place_type_t<T>, std::in_place_type_t<Policy>, Ts&&... ts) -> polymorphic_value<T, Policy>;
  //
  // polymorphic_value creation
  //
  template <class T, class... Ts>
  polymorphic_value<T,detail::Nothing> make_polymorphic_value(Ts&&... ts)
  {
    polymorphic_value<T,detail::Nothing> p;
    p.cb_ = std::make_unique<detail::direct_control_block<T, T>>(
        std::forward<Ts>(ts)...);
    p.ptr_ = p.cb_->ptr();
    return std::move(p);                                         // ljw: WHY MOVE?
  }
  template <class T, class U, class... Ts>
  polymorphic_value<T,detail::Nothing> make_polymorphic_value(Ts&&... ts)
  {
    polymorphic_value<T,detail::Nothing> p;
    p.cb_ = std::make_unique<detail::direct_control_block<T, U>>(
        std::forward<Ts>(ts)...);
    p.ptr_ = p.cb_->ptr();
    return std::move(p);                                         // ljw: WHY MOVE?
  }

  //
  // non-member swap
  //
  template <class T>
  void swap(polymorphic_value<T>& t, polymorphic_value<T>& u) noexcept
  {
    t.swap(u);
  }

  namespace detail
  {
    struct Hashable
    {
      template<class T>
      struct Base
      {
        virtual std::size_t get_hash() = 0;
        virtual const std::type_info& get_contained_typeid() = 0;
        virtual bool compares_equal(T*) = 0;
      };
      template<class CBD>
      struct Custom : public CBD
      {
        std::size_t get_hash() override
        {
        return std::hash<std::decay_t<decltype(*CBD::derived_ptr())>>()(*(this->derived_ptr()));
        }
        const std::type_info& get_contained_typeid() override
        {
          return typeid(CBD);
        }
        //using T = decltype(std::declval<CBD>().ptr());
        using T = typename CBD::BaseT;
        bool compares_equal(T* t) override
        {
          return *(this->derived_ptr()) == *(static_cast<const std::decay_t<decltype(*CBD::derived_ptr())>*>(t));
        }
      };
      template<class Poly>
      struct Immediate
      {
        std::size_t get_hash() const
        {
        return static_cast<const Poly*>(this)->cb_->get_hash();
        }
        const std::type_info& contained_typeid() const
        {
        return static_cast<const Poly*>(this)->cb_->get_contained_typeid();
        }
        bool compares_equal(const Immediate& rhs) const
        {
        return static_cast<const Poly*>(this)->cb_->compares_equal(
          static_cast<const Poly*>(&rhs)->cb_->ptr());
        }
      };
    };

    template<class T>
    bool operator==(const polymorphic_value<T,Hashable>& lhs, const polymorphic_value<T,Hashable>& rhs)
    {
      return lhs.contained_typeid() == rhs.contained_typeid() && lhs.compares_equal(rhs);
    }
    template<class T>
    bool operator!=(const polymorphic_value<T,Hashable>& lhs, const polymorphic_value<T,Hashable>& rhs)
    {
      return !(lhs==rhs);
    }

  }

} // end namespace jbcoe

namespace std
{
  template<class T>
  struct hash<jbcoe::polymorphic_value<T,jbcoe::detail::Hashable>>
  {
    std::size_t operator()(const jbcoe::polymorphic_value<T,jbcoe::detail::Hashable>& p) {return p.get_hash();}
  };
}

#endif
