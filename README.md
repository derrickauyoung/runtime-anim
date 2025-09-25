# Runtime Anim

A repository to hold sample exercises to implement runtime animation concepts, such as skeletons, linear algebra, interpolation, blending, motion matching and more.

In traditional photo-realistic visual effects pipelines, final renders consisted of highly detailed and geometrically dense models, which, coupled with physically-based renderers and cloth, skin, fat and tissue simulations, resulted in day-to-week long render times.

Animation pipelines make use of proxy-asset strategies, but skinning and deformers can be major performance drains, especially at scale.

**Why:** I want to explore real-time performance programming for Animation, as scene and asset complexity rises, to help inform strategies to recover performance for artists working in all production pipelines, from photo-realistic to virtual to games.

**What I tried:** I refreshed myself in a simple skeletal motion exercise in C++, along with a few other related topics like blending and motion matching.

**Takeaways:** Some of the fundamental math principles behind composing joints in a hierarchy were fascinating to (re)learn through code. Many of these techniques more common in Animation programming in Games studios is highly transferable to interactive editing workflows in Visual Effects Animation. When I have time, I must continue learning more!
