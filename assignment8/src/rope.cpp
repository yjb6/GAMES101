#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"
double damping_factor=0/00005;
namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor

        
        for(int i=0;i<num_nodes;i++)
        {
            auto pos=start+(end-start)*i/max(num_nodes-1,1);
            Mass *m=new Mass(pos,node_mass,0);
            masses.push_back(m);
            if(i>0)
            {
                Spring *s=new Spring(masses[i-1],masses[i],k);
                springs.push_back(s);
            }
                
        }
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m2->forces+=s->k*(s->m1->position-s->m2->position).unit()*((s->m1->position-s->m2->position).norm()-s->rest_length);
            s->m1->forces+=s->k*(s->m2->position-s->m1->position).unit()*((s->m2->position-s->m1->position).norm()-s->rest_length);

            //zuni
            s->m1->forces+=-0.005*dot((s->m1->position-s->m2->position).unit(),s->m1->velocity-s->m2->velocity)*(s->m1->position-s->m2->position).unit();
            s->m2->forces+=-0.005*dot((s->m2->position-s->m1->position).unit(),s->m2->velocity-s->m1->velocity)*(s->m2->position-s->m1->position).unit();
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+=gravity;

                // TODO (Part 2): Add global damping
                
                auto a=m->forces/m->mass;
                m->velocity=m->velocity+a*delta_t;
                auto temp_position=m->position;
                m->position=m->position+m->velocity*delta_t;

            }
            m->forces = Vector2D(0, 0);
        }
    }
    

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            s->m2->forces+=s->k*(s->m1->position-s->m2->position).unit()*((s->m1->position-s->m2->position).norm()-s->rest_length);
            s->m1->forces+=s->k*(s->m2->position-s->m1->position).unit()*((s->m2->position-s->m1->position).norm()-s->rest_length);

            // s->m1->forces+=-0.005*dot((s->m1->position-s->m2->position).unit(),s->m1->velocity-s->m2->velocity)*(s->m1->position-s->m2->position).unit();
            // s->m2->forces+=-0.005*dot((s->m2->position-s->m1->position).unit(),s->m2->velocity-s->m1->velocity)*(s->m2->position-s->m1->position).unit();

        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                m->forces+=gravity;
                // TODO (Part 3.1): Set the new position of the rope mass
                // std::cout<<m->position;
                // TODO (Part 4): Add global Verlet damping
                m->position=m->position+(1-damping_factor)*(m->position-m->last_position)+m->forces/m->mass*delta_t*delta_t;
                m->last_position=temp_position;
                
            }
            m->forces=Vector2D(0, 0);;
        }
    }
}
