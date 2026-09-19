using Mono.Cecil;
using System.Text.Json;

// Read-only inspection. Never loads or executes game code.
if (args.Length != 1) throw new ArgumentException("Pass the installed Assembly-CSharp.dll path.");
using var assembly = AssemblyDefinition.ReadAssembly(args[0]);
var needles = new[] { "ApplyControllerForceAtPropeller", "AddForceAtPosition", "inertiaTensor", "centerOfMass", "get_CurrentDrone" };
IEnumerable<TypeDefinition> Walk(IEnumerable<TypeDefinition> types) {
    foreach (var t in types) { yield return t; foreach (var n in Walk(t.NestedTypes)) yield return n; }
}
var types = Walk(assembly.MainModule.Types).ToArray();
var matches = types.SelectMany(t => t.Methods.Where(m => m.HasBody).SelectMany(m =>
    m.Body.Instructions.Where(i => i.Operand is MemberReference r && needles.Any(n => r.Name.Contains(n)))
    .Select(i => new { type = t.FullName, token = t.MetadataToken.ToInt32(), method = m.Name, methodToken = m.MetadataToken.ToInt32(), instruction = i.ToString() }))).ToArray();
var selected = matches.Select(m => m.type).Concat(new[] { "Propeller", "FlightManager", "FramePart", "MotorPart" }).ToHashSet();
var details = types.Where(t => selected.Contains(t.FullName)).Select(t => new {
    name = t.FullName, token = t.MetadataToken.ToInt32(), parent = t.BaseType?.FullName,
    fields = t.Fields.Select(f => new { name=f.Name, type=f.FieldType.FullName, token=f.MetadataToken.ToInt32() }),
    properties = t.Properties.Select(p => new { name=p.Name, type=p.PropertyType.FullName }),
    methods = t.Methods.Select(m => new { name=m.Name, token=m.MetadataToken.ToInt32(), signature=m.FullName })
});
Console.WriteLine(JsonSerializer.Serialize(new { moduleId=assembly.MainModule.Mvid, matches, types=details }, new JsonSerializerOptions { WriteIndented=true }));
