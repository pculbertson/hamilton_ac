function quat_to_rot(q)
    w,x,y,z = q

    return [2*x^2+2*w^2-1 2*(x*y-z*w) 2*(x*z+y*w)
            2*(x*y+w*z) 2*(y^2+w^2)-1 2*(y*z-w*x)
            2*(x*z-y*w) 2*(y*z+w*x) 2*(z^2+w^2)-1]
end

function skew(q)
   return [0 -q[3] q[2]
           q[3] 0 -q[1]
           -q[2] q[1] 0]
end

function Pa(R)
   return 0.5*(R-R') 
end

using Primes

function generateHaltonSamples(dim::Int, n::Int, sf::Array=[1])
  primeNums = primes(1000);

  if length(sf) != dim
    # warn("Scale factors for Halton samples being set to 1 by default!")
    sf = ones(dim)
  end

  samples = zeros(dim,n)

  for (idx, base_val) in enumerate(primeNums[1:dim]) 
    samples[idx, :] = sf[idx] > 0 ? sf[idx]*generateHaltonSequence(n, base_val) : -2*sf[idx]*(generateHaltonSequence(n, base_val) - 0.5)
  end
  samples
end

function generateHaltonSequence(n::Int, base::Int) 
  halton_sequence = zeros(n,1)

  for idx in range(1, stop=n, step=1)
    halton_sequence[idx] = localHaltonSingleNumber(idx, Float64(base))
  end
  halton_sequence
end

function localHaltonSingleNumber(n::Int, b::Float64)
  n0 = n
  hn = 0
  f = 1/b

  while(n0 > 0)
    n1 = floor(n0/b)
    r = n0 - b*n1
    hn = hn + f*r
    f = f/b
    n0 = n1
  end
  hn
end

function align_z(v)
    #function which takes a vector v & returns rotation matrix which aligns z-axis w/ v
    z = [0, 0, 1]
    u = skew(z)*v
    s = norm(u)
    c = v'*z
    
    if s == 0
        if c > 0
            return I
        else
            return Diagonal([-1,-1,1])
        end
    end
    
    R = I + skew(u) + skew(u)*skew(u)*(1-c)/(s^2)
end

function cylinder_grasp_from_normal(v,h,r)
    #takes in an *outward* normal vector, v (from origin), height h, and radius r; 
    #returns portion of grasp matrix Gi corresponding to this normal
    x,y,z = v
    n_z = [0,0,1]
    
    #check intersection w/tops
    if z > 0
        t = h/(2*z) #solve for ray length
        if norm([t*x,t*y]) <= r
            p = t*v
            R = align_z(-n_z)
            return vcat(R,skew(p)*R), R, p
        end
    elseif z < 0
        t = -h/(2*z)
        if norm([t*x,t*y]) <= r
            p = t*v
            R = align_z(n_z)
            return vcat(R,skew(p)*R), R, p
        end
    end
    #if no intersection, then solve for the side
    t = r/norm([x,y])
    p = t*v
    norm_in = -[x,y,0]/norm([x,y,0])
    R = align_z(norm_in)
    return vcat(R,skew(p)*R), R, p
end

function sample_points(N_v,N_h,h=2,r=1,e_noise=0.05,rng=nothing)
  N = N_h * N_v
  eps = 0.025
  u = 2*collect(range(eps,stop=1-eps,length=N_v)) .- 1
  th = 2*pi*collect(range(0,stop=(N_h-1)/(N_h),length=N_h))

  TH = vcat(fill.(th,N_v)...)
  U = repeat(vcat(u,reverse(u)),Int(N_h/2))

  halton_rand = generateHaltonSamples(3,N)
  x = sqrt.(1 .- U.^2).*cos.(TH) .+ e_noise*halton_rand[1,:]
  y = sqrt.(1 .- U.^2).*sin.(TH) .+ e_noise*halton_rand[2,:]
  z = U .+ e_noise*halton_rand[3,:]

  lengths = sqrt.(sum(x.^2 + y.^2 + z.^2, dims=2))
  x = x ./ lengths
  y = y ./ lengths
  z = z ./ lengths

  G = []
  p = []
  for ind in 1:N
    Gr, R, p_i = cylinder_grasp_from_normal([x[ind],y[ind],z[ind]],h,r)
    push!(G,Gr)
    push!(p,p_i)
  end
  G, p
end

function p_norm(x,p,tol)
    #raised to pth power, * 1/p
   return (1/p)*sum(max.(abs.(x),tol*ones(length(x))).^p) 
end

function p_grad(x,p,tol=1e-10)
   fl_abs = max.(abs.(x),tol*ones(length(x)))
   return fl_abs.^(p-2).*(x .+ tol*sign.(x))
end

function pnorm_divergence(y,x,p,tol=1e-10)
    return p_norm(y,p,tol) - p_norm(x,p,tol) - (y-x)'*p_grad(x,p,tol)
end

function pnorm_hessian(x,p,tol=1e-10)
    fl_abs = max.(abs.(x),tol*ones(length(x)))
    norm_diag = (p-1).*fl_abs.^(p-4).*(x.^2 .+ tol*ones(length(x)))
    return Diagonal(norm_diag)
end

#helper function for computing regressor
L(v) = [v[1] v[2] v[3] 0 0 0
     0 v[1] 0 v[2] v[3] 0
     0 0 v[1] 0 v[2] v[3]]


function vee(R)
   return [R[3,2],R[1,3],R[2,1]] 
end